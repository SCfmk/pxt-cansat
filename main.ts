/**
 * Can:Sat - E32 LoRa helper (E32-433T20D style), hardwired pins, AUX-safe for v1.2
 */
//% color=#7C3AED icon="\uf135" block="Can:Sat"
namespace cansat {

    // ---------------------------------------------------------------------
    // Enums MUST be first in MakeCode extensions (order-sensitive compiler)
    // ---------------------------------------------------------------------

    export enum TxMode {
        //% block="transparent"
        Transparent = 0,
        //% block="fixed"
        Fixed = 1
    }

    export enum Power {
        //% block="20 dBm (max)"
        P20dBm = 0,
        //% block="17 dBm"
        P17dBm = 1,
        //% block="14 dBm"
        P14dBm = 2,
        //% block="10 dBm (min)"
        P10dBm = 3
    }

    // Ordered so 1.2 kbps is the “standard choice”
    export enum AirDataRate {
        //% block="1.2 kbps"
        ADR_1_2K = 1,
        //% block="0.3 kbps"
        ADR_0_3K = 0,
        //% block="2.4 kbps"
        ADR_2_4K = 2,
        //% block="4.8 kbps"
        ADR_4_8K = 3,
        //% block="9.6 kbps"
        ADR_9_6K = 4,
        //% block="19.2 kbps"
        ADR_19_2K = 5
    }

    // Ordered so 9600 is the “standard choice”
    export enum UartBaud {
        //% block="9600"
        B9600 = 3,
        //% block="1200"
        B1200 = 0,
        //% block="2400"
        B2400 = 1,
        //% block="4800"
        B4800 = 2,
        //% block="19200"
        B19200 = 4,
        //% block="38400"
        B38400 = 5,
        //% block="57600"
        B57600 = 6,
        //% block="115200"
        B115200 = 7
    }

    export enum Parity {
        //% block="8N1 (none)"
        None = 0,
        //% block="8O1 (odd)"
        Odd = 1,
        //% block="8E1 (even)"
        Even = 2
    }

    // ---------------------------------------------------------------------
    // Hardwired pins for this project
    // ---------------------------------------------------------------------

    const PIN_M0 = DigitalPin.P9
    const PIN_M1 = DigitalPin.P8
    const PIN_AUX = DigitalPin.P16
    const LORA_TX = SerialPin.P14
    const LORA_RX = SerialPin.P15

    // ---------------------------------------------------------------------
    // Internal state
    // ---------------------------------------------------------------------

    let _inited = false
    let _rxHandler: (msg: string) => void = null
    let _receiverInstalled = false

    let _fixedMode = false
    let _useLineMode = true

    // Cache of last known config payload (5 bytes): ADDH, ADDL, CHAN, SPED, OPTION
    let _lastCfg: Buffer = null

    // Track last UART setting so we keep serial.redirect consistent in normal mode
    let _currentUart: UartBaud = UartBaud.B9600

    // Status
    let _lastError = ""
    let _cfgOk = false

    // Debug stage (helps pinpoint where it fails)
    // 0 = idle, 10+ = applyConfig stages, 20+ = readConfig stages
    let _stage = 0

    // ---------------------------------------------------------------------
    // Public blocks (NOT Advanced): init, config, send/receive, tx mode, serial switching
    // ---------------------------------------------------------------------

    //% block="Can:Sat init"
    //% weight=100
    export function init(): void {
        if (_inited) return

        pins.digitalWritePin(PIN_M0, 0)
        pins.digitalWritePin(PIN_M1, 0)
        pins.setPull(PIN_AUX, PinPullMode.PullUp)

        _currentUart = UartBaud.B9600
        serial.redirect(LORA_RX, LORA_TX, baudRateFromUart(_currentUart))

        basic.pause(200)
        waitAuxHigh(4000)

        setWorkModeNormal()
        installReceiverOnce()

        _inited = true
        _lastError = ""
        _cfgOk = true
        _stage = 0
    }

    //% block="apply config ADDH %addh ADDL %addl channel %ch power %p uart %uart parity %parity air %air tx mode %mode"
    //% addh.min=0 addh.max=255 addl.min=0 addl.max=255 ch.min=0 ch.max=255
    //% uart.defl=cansat.UartBaud.B9600
    //% air.defl=cansat.AirDataRate.ADR_1_2K
    //% parity.defl=cansat.Parity.None
    //% p.defl=cansat.Power.P20dBm
    //% mode.defl=cansat.TxMode.Transparent
    //% weight=60
    export function applyConfig(addh: number, addl: number, ch: number, p: Power, uart: UartBaud, parity: Parity, air: AirDataRate, mode: TxMode): void {
        ensureInit()
        _lastError = ""
        _cfgOk = false
        _stage = 10

        const sped = ((parity & 0x03) << 6) | ((uart & 0x07) << 3) | (air & 0x07)

        let option = 0x00
        option |= (p & 0x03)
        if (mode == TxMode.Fixed) option |= (1 << 2)

        _stage = 11
        if (!enterConfigMode()) {
            if (_lastError == "") _lastError = "applyConfig: enter config mode failed"
            exitConfigMode()
            finishFailSafe()
            return
        }

        _stage = 12
        const frame = pins.createBuffer(6)
        frame[0] = 0xC0
        frame[1] = addh & 0xff
        frame[2] = addl & 0xff
        frame[3] = ch & 0xff
        frame[4] = sped & 0xff
        frame[5] = option & 0xff
        serial.writeBuffer(frame)

        _stage = 13
        if (!waitAuxHigh(4000)) {
            _lastError = "applyConfig: AUX timeout after write"
            exitConfigMode()
            finishFailSafe()
            return
        }

        // Verify by reading parameters back (more reliable than echo)
        _stage = 14
        const readback = readParamsWhileInConfigMode()
        exitConfigMode()

        if (readback && readback.length >= 6 && readback[0] == 0xC0) {
            _lastCfg = pins.createBuffer(5)
            for (let i = 0; i < 5; i++) _lastCfg[i] = readback[i + 1]

            _cfgOk = true
            _currentUart = uart
            serial.redirect(LORA_TX, LORA_RX, baudRateFromUart(_currentUart))
            _stage = 0
            return
        }

        _cfgOk = false
        if (_lastError == "") _lastError = "applyConfig: write done but no readback"
        finishFailSafe()
    }

    //% block="read config"
    //% weight=59
    export function readConfig(): void {
        ensureInit()
        _lastError = ""
        _cfgOk = false
        _stage = 20

        _stage = 21
        if (!enterConfigMode()) {
            if (_lastError == "") _lastError = "readConfig: enter config mode failed"
            exitConfigMode()
            finishFailSafe()
            return
        }

        _stage = 22
        const resp = readParamsWhileInConfigMode()
        exitConfigMode()

        _stage = 23
        if (!resp || resp.length < 6) {
            if (_lastError == "") _lastError = "readConfig: no/short response"
            finishFailSafe()
            return
        }

        if (resp[0] == 0xC0 && resp.length >= 6) {
            _lastCfg = pins.createBuffer(5)
            for (let i = 0; i < 5; i++) _lastCfg[i] = resp[i + 1]
            _cfgOk = true
            _stage = 0
            return
        }

        _cfgOk = false
        _lastError = "readConfig: unexpected header " + resp[0]
        finishFailSafe()
    }

    //% block="send string %msg"
    //% weight=50
    export function sendString(msg: string): void {
        ensureInit()

        if (_useLineMode) msg = msg + "\n"

        if (_fixedMode) {
            let addh = 0x00
            let addl = 0x00
            let ch = 0x17
            if (_lastCfg && _lastCfg.length >= 3) {
                addh = _lastCfg[0]
                addl = _lastCfg[1]
                ch = _lastCfg[2]
            }

            const hdr = pins.createBuffer(3)
            hdr[0] = addh
            hdr[1] = addl
            hdr[2] = ch
            serial.writeBuffer(hdr)
        }

        serial.writeString(msg)
    }

    //% block="on receive string"
    //% weight=49
    export function onReceiveString(handler: (msg: string) => void): void {
        _rxHandler = handler
        installReceiverOnce()
    }

    //% block="set tx mode %mode"
    //% weight=80
    export function setTxMode(mode: TxMode): void {
        _fixedMode = (mode == TxMode.Fixed)
    }

    //% block="serial to LoRa"
    //% weight=70
    export function serialToLoRa(): void {
        ensureInit()
        serial.redirect(LORA_TX, LORA_RX, baudRateFromUart(_currentUart))
    }

    //% block="serial to USB"
    //% weight=69
    export function serialToUSB(): void {
        serial.redirectToUSB()
    }

    // ---------------------------------------------------------------------
    // Advanced ("More") blocks
    // ---------------------------------------------------------------------

    //% block="config ok?"
    //% advanced=true
    export function configOk(): boolean {
        return _cfgOk
    }

    //% block="last error"
    //% advanced=true
    export function lastError(): string {
        return _lastError
    }

    //% block="debug stage"
    //% advanced=true
    export function debugStage(): number {
        return _stage
    }

    //% block="config ADDH"
    //% advanced=true
    export function cfgADDH(): number {
        return (_lastCfg && _lastCfg.length >= 1) ? _lastCfg[0] : 0
    }

    //% block="config ADDL"
    //% advanced=true
    export function cfgADDL(): number {
        return (_lastCfg && _lastCfg.length >= 2) ? _lastCfg[1] : 0
    }

    //% block="config channel"
    //% advanced=true
    export function cfgChannel(): number {
        return (_lastCfg && _lastCfg.length >= 3) ? _lastCfg[2] : 0
    }

    //% block="AUX is high"
    //% advanced=true
    export function auxIsHigh(): boolean {
        return pins.digitalReadPin(PIN_AUX) == 1
    }

    //% block="use line framing %enabled"
    //% advanced=true
    export function useLineFraming(enabled: boolean): void {
        _useLineMode = enabled
    }

    //% block="send fixed to ADDH %addh ADDL %addl CH %ch msg %msg"
    //% addh.min=0 addh.max=255 addl.min=0 addl.max=255 ch.min=0 ch.max=255
    //% advanced=true
    export function sendFixed(addh: number, addl: number, ch: number, msg: string): void {
        ensureInit()
        if (_useLineMode) msg = msg + "\n"
        const hdr = pins.createBuffer(3)
        hdr[0] = addh & 0xff
        hdr[1] = addl & 0xff
        hdr[2] = ch & 0xff
        serial.writeBuffer(hdr)
        serial.writeString(msg)
    }

    // ---------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------

    function finishFailSafe(): void {
        // Guarantee a non-empty error whenever configOk is false.
        if (!_cfgOk && _lastError == "") {
            _lastError = "Failed (stage " + _stage + ")"
        }
    }

    function ensureInit(): void {
        if (!_inited) init()
    }

    function setWorkModeNormal(): void {
        pins.digitalWritePin(PIN_M0, 0)
        pins.digitalWritePin(PIN_M1, 0)
        basic.pause(30)
        waitAuxHigh(4000)
        serial.redirect(LORA_TX, LORA_RX, baudRateFromUart(_currentUart))
    }

    function enterConfigMode(): boolean {
        pins.digitalWritePin(PIN_M0, 1)
        pins.digitalWritePin(PIN_M1, 1)
        basic.pause(80)

        if (!waitAuxHigh(5000)) {
            _lastError = "Timeout entering config mode (AUX)"
            return false
        }

        // Config interface: 9600 8N1
        serial.redirect(LORA_TX, LORA_RX, BaudRate.BaudRate9600)
        return true
    }

    function exitConfigMode(): void {
        setWorkModeNormal()
    }

    function waitAuxHigh(timeoutMs: number): boolean {
        const start = control.millis()
        while (control.millis() - start < timeoutMs) {
            if (pins.digitalReadPin(PIN_AUX) == 1) return true
            basic.pause(5)
        }
        return false
    }

    function readParamsWhileInConfigMode(): Buffer {
        // Clear any pending input
        serial.readString()

        const cmd = pins.createBuffer(3)
        cmd[0] = 0xC1
        cmd[1] = 0xC1
        cmd[2] = 0xC1
        serial.writeBuffer(cmd)

        if (!waitAuxHigh(3000)) {
            if (_lastError == "") _lastError = "Timeout waiting AUX after read cmd"
            return null
        }

        const resp = readBytesWithTimeout(6, 1200)
        if (!resp || resp.length == 0) {
            if (_lastError == "") _lastError = "No bytes received from module"
            return null
        }
        return resp
    }

    function readBytesWithTimeout(n: number, timeoutMs: number): Buffer {
        const start = control.millis()
        let buf = pins.createBuffer(0)

        while (control.millis() - start < timeoutMs && buf.length < n) {
            const b = serial.readBuffer(n - buf.length)
            if (b && b.length > 0) {
                buf = bufferConcat(buf, b)
            } else {
                basic.pause(10)
            }
        }

        if (buf.length == 0) return null
        return buf
    }

    function bufferConcat(a: Buffer, b: Buffer): Buffer {
        const out = pins.createBuffer(a.length + b.length)
        for (let i = 0; i < a.length; i++) out[i] = a[i]
        for (let j = 0; j < b.length; j++) out[a.length + j] = b[j]
        return out
    }

    function baudRateFromUart(u: UartBaud): BaudRate {
        switch (u) {
            case UartBaud.B1200: return BaudRate.BaudRate1200
            case UartBaud.B2400: return BaudRate.BaudRate2400
            case UartBaud.B4800: return BaudRate.BaudRate4800
            case UartBaud.B9600: return BaudRate.BaudRate9600
            case UartBaud.B19200: return BaudRate.BaudRate19200
            case UartBaud.B38400: return BaudRate.BaudRate38400
            case UartBaud.B57600: return BaudRate.BaudRate57600
            case UartBaud.B115200: return BaudRate.BaudRate115200
            default: return BaudRate.BaudRate9600
        }
    }

    function installReceiverOnce(): void {
        if (_receiverInstalled) return
        _receiverInstalled = true

        serial.onDataReceived(serial.delimiters(Delimiters.NewLine), function () {
            const s = serial.readString()
            if (_rxHandler) _rxHandler(s)
        })
    }
}
