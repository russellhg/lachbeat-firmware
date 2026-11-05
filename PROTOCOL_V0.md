# Latchbeat Bus Module Protocol Specification

## 1. Physical Layer & Addressing (`lb_bus.h`)

The **LatchBeat Module Bus** connects the **Leader MCU (`ESP32-S3` / `ESP-32S`)** to up to **24 hot-swappable Follower Modules (`Arduino Pro Mini 3.3V` / `CH32V003` / `STM32G0`)** over a shared `3.3V` Fast-Mode `I2C` bus (`100–400 kHz`) paired with an active-low open-drain interrupt line (`ALERT#`).

| Bus Parameter | Specification | Engineering Rationale | 
| ----- | ----- | ----- | 
| **Shared Signals (5-Wire Core)** | `GND`, `3V3_MOD`, `SDA`, `SCL`, `ALERT#` (plus optional `DETECT`) | Uses native `I2C` hardware on every MCU with zero external transceiver ICs | 
| **Unassigned Boot Address** | `0x30` (`lb::BOOT_ADDR`) | Every newly powered or hot-plugged module listens at `0x30` until assigned | 
| **Dynamic Session Pool** | `0x10` – `0x27` (`24` slots, `lb::SESSION_FIRST`..`LAST`) | Avoids `I2C` reserved (`0x00–0x07`, `0x78–0x7F`), SMBus, and `MPU-6050` (`0x68`) addresses | 
| **Checksum Algorithm** | `CRC-8/SMBUS` (polynomial `0x07`, init `0x00`) | Detects multi-follower boot collisions and hot-unplug mid-byte corruption | 

## 2. Dynamic Discovery & Collision Backoff State Machine

Because a player can plug in multiple identical modules (e.g., two button modules or two fader modules) or power on the whole deck with 5 modules already attached, discovery uses **64-bit Unique IDs (`UID`) with randomized backoff**:

* **Step 1 — Staggered Boot Wait (`ST_WAIT_START`)**: On power-up, each Follower seeds a PRNG from its 8-byte `UID` + `micros()`, waits a random **`10–200 ms`**, joins `I2C` at `0x30`, and pulls `ALERT#` low (`ST_UNASSIGNED`).

* **Step 2 — Boot Frame Read (`12 Bytes` from `0x30`)**: When `ALERT#` goes low (or every `250 ms`), the Leader reads `12 bytes` from `0x30`:

| Byte Offset | Field Name | Value / Type | Description | 
| ----- | ----- | ----- | ----- | 
| **Byte `0`** | `MAGIC` | `'L'` (`0x4C`) | Protocol magic header byte | 
| **Byte `1`** | `MODULE_TYPE` | `0x01`=Button, `0x02`=Pot, `0x03`=Encoder, `0x04`=Fader | Identifies module control topology | 
| **Byte `2`** | `FW_VERSION` | `0x01` | Follower firmware version | 
| **Bytes `3..10`** | `UID[8]` | 64-bit hardware or EEPROM UID | Unique serial number of the Follower module | 
| **Byte `11`** | `CRC8` | `crc8(bytes 0..10)` | Validates frame integrity and detects bus collisions | 

* **Step 3A — Collision Recovery (`CMD_BACKOFF = 0xB0`)**: If two unassigned modules answer at `0x30` simultaneously, their wired-AND `SDA` bytes corrupt the `CRC8`. The Leader broadcasts `0xB0` (`CMD_BACKOFF`) to `0x30`; all unassigned modules release `ALERT#`, stop `I2C`, wait a new random **`20–300 ms`**, and retry one by one.

* **Step 3B — Address Assignment (`CMD_ASSIGN = 0xA1`)**: On a valid `CRC8`, the Leader picks a free session address (`0x10..0x27`, or reuses the module's previous session address if re-plugged) and writes an 11-byte command to `0x30`: `[0xA1, UID[0..7], new_addr, crc8]`. Only the Follower whose `UID` matches switches its `I2C` peripheral to `new_addr` (`ST_ASSIGNED`).

* **Step 4 — Verification Read (`REG_IDENT = 0x00`)**: After `10 ms`, the Leader reads `REG_IDENT` (`0x00`, 12 bytes) at `new_addr` to confirm the `UID` matches before marking the slot active.

## 3. Real-Time Control Event Format (`REG_EVENTS = 0x10`)

When an assigned Follower detects a debounced button edge (`5 ms` settle), potentiometer change (`±8 LSB` hysteresis), or rotary encoder step, it increments `evSeq`, updates its event buffer, and pulls `ALERT#` low. The Leader writes register pointer `0x10` (`REG_EVENTS`) and reads **4 bytes**:

| Byte Offset | Field Name | Value / Encoding | Leader Action | 
| ----- | ----- | ----- | ----- | 
| **Byte `0`** | `evSeq` | `0x00–0xFF` rolling counter | Skips processing if `evSeq == lastSeq` (no change since last read) | 
| **Byte `1`** | `evButtons` | Bitmask (`bit 0` = primary switch) | Sends MIDI `Note-On` (`0x90`) on `0->1` and records note in Tracking Table; sends `Note-Off` (`0x80`) on `1->0` | 
| **Byte `2`** | `evValueHi` | Upper byte of 10-bit control (`0..1023`) | Combined with `evValueLo` and right-shifted `>> 3` to produce 7-bit MIDI `CC` (`0..127`) | 
| **Byte `3`** | `evValueLo` | Lower byte of 10-bit control (`0..1023`) | Follower releases `ALERT#` automatically once all 4 bytes are read | 

## 4. Hot-Unplug Detection, Stuck-Bus Recovery & Panic Safety

* **Heartbeat Removal Detection (`150 ms` max)**: The Leader polls `REG_EVENTS` on all present modules whenever `ALERT#` is low and at least once every `50 ms`. If a module fails to ACK **`3` consecutive times (`MISS_LIMIT = 3`)**, the Leader marks it removed and **immediately sends `Note-Off (0x80)` for any note held by that module (`heldNote`)**, preventing stuck notes.

* **Stuck `SDA` Bus Recovery (`UM10204 §3.1.16`)**: If a module is yanked mid-byte while pulling `SDA` low (`busErrors >= 5`), the Leader temporarily switches `SCL` to open-drain GPIO, pulses up to **9 clock cycles** until `SDA` releases high, issues a `STOP` condition, and restarts the `I2C` hardware controller without rebooting the ESP32.
