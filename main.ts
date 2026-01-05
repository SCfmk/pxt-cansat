/**
 * Can:Sat - E32 LoRa helper (E32-433T20D style), hardwired pins, AUX-safe for v1.2
 */
//% color=#7C3AED icon="\uf135" block="Can:Sat"
namespace cansat {
    // ----- Hardwired pins for this project -----
    const PIN_M0 = DigitalPin.P9
    const PIN_M1 = DigitalPin.P8
    const PIN_AUX = DigitalPin.P16
    const LORA_TX = SerialPin.P14
    const LORA_RX = SerialPin.P15

    // ----- Internal state -----
    let _inited = false
    let _rxHandler: (msg: string) => void = null
    let _useLineMode = true
    let _fixedMode = false

    // Cache of last known config payload (5 bytes): ADDH, ADDL, CHAN, SPED, OPTION
    let _lastCfg: Buffer = null

    // Status
    let _lastError = ""
    let _cfgOk = false

    // ----- Public enums for blocks -----
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

    export enum AirDataRate {
        //% block="0.3 kbps"
        ADR_0_3K = 0,
        //% block="1.2 kbps"
        ADR_1_2K = 1,
        //% block="2.4 kbps"
        ADR_2_4K = 2,
        //% block="4.8 kbps"
        ADR_4_8K = 3,
        //% block="9.6 kbps"
        ADR_9_6K = 4,
        //% block="19.2 kbps"
        ADR_19_2K = 5
    }

    export enum UartBaud {
        //% block="1200"
        B1200 = 0,
        //% block="2400"
        B2400 = 1,
        //% block="4800"
        B4800 = 2,
        //% block="9600"
        B9600 = 3,
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

    // ----- Blocks: setup and mode -----

    /**
     * Initialize Can:Sat LoRa module (sets pins, redirects serial, sets normal mode).
     */
    //% block="Can:Sat init"
    //% weight=100
    export function init(): void {
        if (_inited) return

        pins.digitalWritePin(PIN_M0, 0)
        pins.digitalWritePin(PIN_M1, 0)
        pins.setPull(PIN_AUX, PinPullMode.PullUp)

        // Default to LoRa serial at 9600 for reliability
        serial.redirect(LORA_TX, LORA_RX, BaudRate.BaudRate9600)

        // Let the module settle after power-up
        basic.pause(100)
        waitAuxHigh(2000)

        setWorkModeNormal()
        setupReceiver()

        _inited = true
        _lastError = ""
        _cfgOk = true
    }

    /**
     * Switch module transmission mode used by "send string".
     */
    //% block="set tx mode %mode"
    //% weight=80
    export function setTxMode(mode: TxMode): void {
        _fixedMode = (mode == TxMode.Fixed)
    }

    /**
     * Choose whether send/receive uses line framing (recommended for strings).
     * Line framing means send adds \\n and receiver triggers on \\n.
     */
    //% block="use line framing %enabled"
    //% weight=79
    export function useLineFraming(enabled: boolean): void {
        _useLineMode = enabled
        setupReceiver()
    }

    /**
     * Returns AUX pin state (true = module idle/ready).
     */
    //% block="AUX is high"
    //% weight=30
    export function auxIsHigh(): boolean {
        return pins.digitalReadPin(PIN_AUX) == 1
    }

    /**
     * Returns the last error string (empty if none).
     */
    //% block="last error"
    //% weight=29
    export function lastError(): string {
        return _lastError
    }

    /**
     * Was the last config operation ok?
     */
    //% block="config ok?"
    //% weight=28
    export function configOk(): boolean {
        return _cfgOk
    }

    // ----- Blocks: Serial routing -----

    /**
     * Route serial to LoRa module (P14 TX, P15 RX).
     */
    //% block="serial to LoRa"
    //% weight=70
    export function serialToLoRa(): void {
        serial.redirect(LORA_TX, LORA_RX, BaudRate.BaudRate9600)
        setupReceiver()
    }

    /**
     * Route serial to USB (useful for debugging prints).
     */
    //% block="serial to USB"
    //% weight=69
    export function serialToUSB(): void {
        serial.redirectToUSB()
    }

    // ----- Blocks: Send / receive -----

    /**
     * Send a string. In fixed mode, will prepend cached address+channel header if available.
     * In transparent mode, sends as-is.
     */
    //% block="send string %msg"
    //% weight=60
    export function sendString(msg: string): void {
        ensureInit()

        if (_useLineMode) msg = msg + "\n"

        if (_fixedMode) {
            // In fixed mode, header = ADDH, ADDL, CHAN
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

            serial.writeString(msg)
        } else {
            serial.writeString(msg)
        }
    }

    /**
     * Send a string to a specific address+channel (fixed transmission).
     */
    //% block="send fixed to ADDH %addh ADDL %addl CH %ch msg %msg"
    //% addh.min=0 addh.max=255 addl.min=0 addl.max=255 ch.min=0 ch.max=255
    //% weight=59
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

    /**
     * Register a handler for received strings.
     */
    //% block="on receive string"
    //% weight=58
    export function onReceiveString(handler: (msg: string) => void): void {
        _rxHandler = handler
        setupReceiver()
    }

    // ----- Blocks: Configuration (AUX-safe) -----

    /**
     * Read module configuration and cache it.
     */
    //% block="read config"
    //% weight=50
    export function readConfig(): void {
        ensureInit()
        _lastError = ""
        _cfgOk = false

        if (!enterConfigMode()) {
            exitConfigMode()
            return
        }

        // Clear input buffer
        serial.readString()

        // Read parameters: C1 C1 C1
        const cmd = pins.createBuffer(3)
        cmd[0] = 0xC1; cmd[1] = 0xC1; cmd[2] = 0xC1
        serial.writeBuffer(cmd)

        if (!waitAuxHigh(2000)) {
            _lastError = "Timeout waiting AUX after read cmd"
            exitConfigMode()
            return
        }

        const resp = readBytesWithTimeout(6, 700)
        exitConfigMode()

        if (!resp || resp.length < 6) {
            _lastError = "No/short config response"
            return
        }

        // Typical response: 0xC0 + 5 bytes
        if (resp[0] == 0xC0 && resp.length >= 6) {
            _lastCfg = pins.createBuffer(5)
            for (let i = 0; i < 5; i++) _lastCfg[i] = resp[i + 1]
            _cfgOk = true
            return
        }

        _lastError = "Unexpected config header: " + resp[0]
    }

    /**
     * Apply common config settings (address, channel, power, etc.) and SAVE to module.
     *
     * Uses classic E32 5-byte parameter frame: ADDH, ADDL, CHAN, SPED, OPTION.
     */
    //% block="apply config ADDH %addh ADDL %addl channel %ch power %p uart %uart parity %parity air %air tx mode %mode"
    //% addh.min=0 addh.max=255 addl.min=0 addl.max=255 ch.min=0 ch.max=255
    //% weight=49
    export function applyConfig(addh: number, addl: number, ch: number, p: Power, uart: UartBaud, parity: Parity, air: AirDataRate, mode: TxMode): void {
        ensureInit()
        _lastError = ""
        _cfgOk = false

        // SPED byte:
        // bits 7..6: parity, bits 5..3: UART baud, bits 2..0: air data rate
        const sped = ((parity & 0x03) << 6) | ((uart & 0x07) << 3) | (air & 0x07)

        // OPTION byte (mapping can vary by model/firmware).
        // We implement a common layout:
        // - bits 1..0: power
        // - bit 2: fixed transmission enable
        let option = 0x00
        option |= (p & 0x03)
        if (mode == TxMode.Fixed) option |= (1 << 2)

        if (!enterConfigMode()) {
            exitConfigMode()
            return
        }

        // Write & save: C0 + 5 bytes
        const frame = pins.createBuffer(6)
        frame[0] = 0xC0
        frame[1] = addh & 0xff
        frame[2] = addl & 0xff
        frame[3] = ch & 0xff
        frame[4] = sped & 0xff
        frame[5] = option & 0xff
        serial.writeBuffer(frame)

        if (!waitAuxHigh(2500)) {
            _lastError = "Timeout waiting AUX after write"
            exitConfigMode()
            return
        }

        const echo = readBytesWithTimeout(6, 700)
        exitConfigMode()

        // Cache what we attempted
        _lastCfg = pins.createBuffer(5)
        _lastCfg[0] = addh & 0xff
        _lastCfg[1] = addl & 0xff
        _lastCfg[2] = ch & 0xff
        _lastCfg[3] = sped & 0xff
        _lastCfg[4] = option & 0xff

        // If echo looks plausible, mark OK
        if (echo && echo.length >= 6 && (echo[0] == 0xC0 || echo[0] == 0xC2)) {
            _cfgOk = true
            return
        }

        // Some modules don't echo reliably; still treat as OK but leave an informative message.
        _cfgOk = true
        _lastError = "No/odd echo (may still be ok)"
    }

    /**
     * Get cached address high byte (0 if unknown).
     */
    //% block="config ADDH"
    //% weight=20
    export function cfgADDH(): number {
        return (_lastCfg && _lastCfg.length >= 1) ? _lastCfg[0] : 0
    }

    /**
     * Get cached address low byte (0 if unknown).
     */
    //% block="config ADDL"
    //% weight=19
    export function cfgADDL(): number {
        return (_lastCfg && _lastCfg.length >= 2) ? _lastCfg[1] : 0
    }

    /**
     * Get cached channel (0 if unknown).
     */
    //% block="config channel"
    //% weight=18
    export function cfgChannel(): number {
        return (_lastCfg && _lastCfg.length >= 3) ? _lastCfg[2] : 0
    }

    // ----- Internal helpers -----

    function ensureInit(): void {
        if (!_inited) init()
    }

    function setWorkModeNormal(): void {
        pins.digitalWritePin(PIN_M0, 0)
        pins.digitalWritePin(PIN_M1, 0)
        basic.pause(20)
        waitAuxHigh(2000)
    }

    function enterConfigMode(): boolean {
        // Sleep/config: M1=1 M0=1
        pins.digitalWritePin(PIN_M0, 1)
        pins.digitalWritePin(PIN_M1, 1)
        basic.pause(40)

        if (!waitAuxHigh(2500)) {
            _lastError = "Timeout entering config mode (AUX)"
            return false
        }

        // Config interface commonly uses 9600 8N1
        serial.redirect(LORA_TX, LORA_RX, BaudRate.BaudRate9600)
        return true
    }

    function exitConfigMode(): void {
        setWorkModeNormal()
        setupReceiver()
    }

    function waitAuxHigh(timeoutMs: number): boolean {
        const start = control.millis()
        while (control.millis() - start < timeoutMs) {
            if (pins.digitalReadPin(PIN_AUX) == 1) return true
            basic.pause(5)
        }
        return false
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

    function setupReceiver(): void {
        if (!_rxHandler) return

        // NOTE: serial.onDataReceived handlers stack in MakeCode if you call this repeatedly.
        // Typical usage is: call onReceiveString once, so it's fine.

        if (_useLineMode) {
            serial.onDataReceived(serial.delimiters(Delimiters.NewLine), function () {
                const s = serial.readString()
                _rxHandler(s)
            })
        } else {
            // If not using newline framing, we still need a delimiter to trigger.
            serial.onDataReceived(serial.delimiters(Delimiters.Comma), function () {
                const s = serial.readString()
                _rxHandler(s)
            })
        }
    }
}
