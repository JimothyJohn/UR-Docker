# Bill of materials: RealSense pick kit for UR e-Series (PolyScope 5)

**Prices checked 2026-09-28; the PC lines (K1, K1b, K1c, K2) re-sourced 2026-09-29 when the kit moved from a RevPi Connect 5 to a Raspberry Pi 4.** All prices are USD, per unit, qty 1, excluding tax, shipping and duty, unless the line says otherwise. EUR prices are converted at the ECB reference rate for 2026-09-28, **1 EUR = 1.1378 USD** ([ECB daily XML](https://www.ecb.europa.eu/stats/eurofxref/eurofxref-daily.xml)). "Not found" means I could not verify the number from a primary page or a distributor listing. It is not an estimate. Where a number came only from a search-engine result snippet (the page itself blocked the fetcher), the line says so.

**What the kit is.** A Raspberry Pi 4 (arm64, minimal Debian, no GPU; Nick, 2026-09-29: keep the compute efficient, a Pi 4 is enough) runs the RGB-D cockpit and the pick server. The industrial CM5 boxes it replaced stay as alternatives (§K1-alt). It talks to the UR controller over Ethernet. One Intel/RealSense D435 hangs on the tool flange in the printed adapter (`hardware/d435-tool-bracket/`) and connects to the PC over USB 3. The reference cell is a UR3e with a Robotiq Hand-E.

---

## 1. Summary

### Kit per cell (primary configuration)

| # | Line | Part number | Qty | Unit price now | Extended |
|---|------|-------------|-----|----------------|----------|
| K1 | Camera PC: Raspberry Pi 4 Model B, 4 GB | SC0194 | 1 | $120.00 | $120.00 |
| K1b | DIN-rail aluminium case for the Pi 4, with fan and heatsinks | Waveshare PI4-CASE-DIN-RAIL-A | 1 | $9.99 | $9.99 |
| K1c | Industrial microSD, 32 GB, MLC | SanDisk SDSDQAF3-032G-I | 1 | $30.43 | $30.43 |
| K2 | DIN-rail PSU, 5 V 3 A 15 W | Mean Well HDR-30-5 | 1 | $19.73 | $19.73 |
| K3 | Depth camera: RealSense D435 | 82635AWGDVKPRQ | 1 | $314.00 | $314.00 |
| K4 | High-flex USB 3.2 A to C cable, dual screw-lock, 3 m | Newnex U3HLA01C12-030 | 1 | $188.00 | $188.00 |
| K5 | Shielded Cat6 patch cable, 10 ft (3.0 m) | L-com TRD695SCR-BLK-10 | 1 | $42.19 | $42.19 |
| K6 | Cable management: hook-and-loop, 1/2 in x 25 yd roll | VELCRO Brand ONE-WRAP, black, 189755 | 1 roll | $50.00 | $50.00 |
| K7 | Bracket print material: PPA-CF, about 40 g per e-Series bracket (Rev C, estimated) | Bambu Lab PPA-CF N06-K0-1.75-750-SPL | 40 g | $0.20/g ($149.99 per 0.75 kg) | $8.00 |
| K8 | Camera screw: 1/4-20 UNC x 3/8 in flat head socket cap, 82 deg, 18-8 SS (Rev C: the only camera screw) | U-Turn Fasteners, 3/8 in length of the K8 line below (part number not checked) | 1 | not checked | not checked |
| K10 | Tool bolts through the adapter: M6 low-head socket cap, DIN 7984, A2 (length: see §2 K10) | Monster Bolts "SHCS Low - SA2 M06 x 016.0010" (10-pack) | 4 (1 pack) | $3.05 / 10 | $3.05 |
| K11 | Dowel pin, dia. 6 m6 x 20 (robot side, through the adapter's pin slot) | ISO 8734, 6 m6 x 20 | 1 | not found | not found |
| K12 | Medium-strength threadlocker (Loctite 243 class) | — | small amount | not found | not found |
| | **Kit subtotal (priced lines)** | | | | **$785.39** |

With an **industrial PC** in place of K1, K1b, K1c and K2 (the 24 V HDR-60-24, $20.10, replaces the 5 V supply): KUNBUS RevPi Connect 5 100416 ($796.00) makes the kit **$1,421.34**; CompuLab IOT-GATE-RPI5 8 GB / 64 GB eMMC ($485.00 plus its $2.50 DIN clip) makes it **$1,112.84**. See §K1-alt for when that is worth it.

K8 (Rev C's 3/8 in screw), K11 and K12 are real parts that are not in any subtotal because I could not verify a price. All three cost a few dollars. The subtotal also leaves out the RealSense store's tariff surcharge (in force since 2026-02-03, amount not published on the product page), shipping, and US import duty on the EUR-sourced RevPi if you buy direct from KUNBUS.

### Customer-supplied / reference (not in the kit)

| # | Line | Part number | Qty | Unit price now | Extended |
|---|------|-------------|-----|----------------|----------|
| C1 | Universal Robots UR3e cobot arm | UR3e | 1 | $30,852.00 (distributor listing; UR publishes no list price) | $30,852.00 |
| C2 | Robotiq Hand-E gripper kit for e-Series | HND-ES-UR-KIT | 1 | $8,416.41 | $8,416.41 |
| | **Customer-supplied subtotal** | | | | **$39,268.41** |

### Optional / upgrade lines (not in any subtotal)

| # | Line | Part number | Unit price now | Use it when |
|---|------|-------------|----------------|-------------|
| O1 | igus triflex R cobot dress-pack kit for UR3(e), size 30 | TRE.918.067.LAE30 | EUR 241.15 (about $274.38) | You want a real e-chain along the arm instead of hook-and-loop |
| O2 | Newnex active high-flex USB 3.2 A to C, screw-lock, 5 m | ULHLU11AC12-5M | $399.00 | The PC sits more than 3 m of cable run from the camera |
| O3 | Newnex high-flex USB 3.2 A to C, dual screw-lock, 5 m (passive) | U3HLA01C12-050 | $228.00 | 5 m passive. Newnex rates it for only 1 m of flexing length and 1.5 A. Prefer O2 at 5 m. |
| O4 | Newnex high-flex A to right-angle C, dual screw-lock, 1 m | U3HLA01C6D-010 | $159.00 | Right-angle plug at the camera end (the 3 m and 5 m right-angle variants exist; price not found) |
| O5 | RealSense D405 (short range, 7 to 50 cm) | D405 | $272.00 | Evaluation only. The current bracket and code target the D435. |

---

## 2. Per-line detail

### K1: Camera PC (primary): Raspberry Pi 4 Model B, 4 GB (SC0194)

- **Why this one (2026-09-29).** The cockpit and pick server are pure stdlib Python on top of librealsense's C++ pipeline; nothing in the kit needs a Pi 5, and Nick asked to keep the compute efficient. The Pi 4 has **2 x USB 3.0** ports (the D435's 848x480 @ 30 stream needs USB 3, see *Candidates screened*), Gigabit Ethernet, a quad Cortex-A72 at 1.8 GHz and 1 to 8 GB of LPDDR4 ([RPi specifications](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/)). Raspberry Pi's [power documentation](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc) gives it **1.2 A total for USB peripherals** on the recommended 3 A supply and a typical bare-board draw of 600 mA. That is more USB current than a Pi 5 gets from a 3 A supply (600 mA), and it covers the D435 (the RevPi datasheet budgets 900 mA per port for it).
- **What you give up against the RevPi Connect 5.** A slower SoC (Cortex-A72 at 1.8 GHz vs the Pi 5 / CM5's Cortex-A76 at 2.4 GHz): the librealsense filters and alignment are C++ and fine, the Python side (PNG per request, the volume detector) will take roughly two to three times longer per frame. The pick node asks for a frame only at FIND / LOOK / REFINE, so that is cycle time, not correctness. **Not measured on either board** as of 2026-09-29; `deploy/pi/README.md` records the first run when it happens. Also: microSD instead of eMMC (K1c is an industrial MLC card), one Ethernet port (the cell subnet is the only one the cockpit needs), 5 V input instead of 24 V (K2 changes with it), a commercial-grade board rather than a -25 °C rated cabinet PC.
- **Why 4 GB.** `deploy/pi/install.sh` compiles librealsense with jobs sized from RAM and adds a swapfile under 4 GiB; 2 GB builds but slowly, 1 GB is not worth the wait. 8 GB buys nothing for this workload and costs $50 more since 2026-04-01.
- **Pi 3: rejected.** Every Pi 3 is USB 2.0 only ([3B+ page](https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/): "4 USB 2.0 ports", Cortex-A53 at 1.4 GHz, 1 GB). The cockpit would negotiate the D435 down to 640x480 @ 15, which no number in the pick kit was tuned at.
- **Official price.** $55 at launch (2019-06-24, [RPi news](https://www.raspberrypi.com/news/raspberry-pi-4-on-sale-now-from-35/)); +$15 on 2026-02-02 (4 GB density, [RPi news](https://www.raspberrypi.com/news/more-memory-driven-price-rises/)); +$25 on 2026-04-01 ([RPi news](https://www.raspberrypi.com/news/a-new-3gb-raspberry-pi-4-for-83-75-and-more-memory-driven-price-increases/)) = **$95**. The 2025-10-01 rise did not touch the Pi 4. The product page shows only "From $35".
- **Distributor price (used in the total).** [Adafruit 4296](https://www.adafruit.com/product/4296): **$120.00, in stock**, 2026-09-29. Also [PiShop.us](https://www.pishop.us/product/raspberry-pi-4-model-b-4gb/) $100.00 (stock not shown on the page); SparkFun $120.00 and Micro Center $94.99 (search snippets, 2026-09-29; the Micro Center page returned 403). Buy at the $95 to $100 sources if they have stock; $120 is the price I could verify in stock.

### K1b: DIN-rail case: Waveshare PI4-CASE-DIN-RAIL-A

- Aluminium, sand-blasted, with a cooling fan and heatsinks; openings for the USB, Ethernet and USB-C power connectors. [Waveshare](https://www.waveshare.com/pi4-case-din-rail-a.htm): **$9.99**, 2026-09-29. The fan matters: the case sits in a cabinet next to a mains supply, and the Pi's firmware throttles the SoC at its default `temp_limit` of 85 °C ([RPi config.txt docs](https://www.raspberrypi.com/documentation/computers/config_txt.html)).
- Alternatives: Waveshare's ABS DIN case ($3.99, no fan), Hitaltech's Railbox / Modulbox 4M cases (GBP 10.50 to 16 at The Pi Hut), CamdenBoss (GBP 9). Prices from search snippets, 2026-09-29.
- History: not found.

### K1c: Boot media: SanDisk Industrial microSD 32 GB, MLC (SDSDQAF3-032G-I)

- The Pi 4 boots from microSD. A consumer card is the usual point of failure in a cabinet; this one is SanDisk's industrial MLC line (the temperature rating is on SanDisk's datasheet; the product page did not render it for the fetcher). 32 GB is plenty (Debian minimal + the librealsense build tree wants about 5 GiB).
- **Price:** [Mouser](https://www.mouser.com/en/ProductDetail/SanDisk/SDSDQAF3-032G-I?qs=1mbolxNpo8edRuxnvUaEMw%3D%3D) **$30.43**, 2,075 in stock, 2026-09-29 (search snippet; the page timed out for the fetcher). eBay listings at about $22 exist; not a source for a kit.
- History: not found.

### K1-alt: Industrial PC (alternative): KUNBUS Revolution Pi RevPi Connect 5

Pick this instead of K1 + K1b + K1c + K2 when the cell needs a 24 V cabinet PC with eMMC, two Ethernet ports and a -25 to +60 °C rating. It was the primary line until 2026-09-29.

- **Why this one.** It has two **USB-A 3.2 Gen 1 (5 Gbit/s)** ports. It takes **24 V DC (10.8 to 28.8 V)** and mounts on a DIN rail. It has two Gigabit Ethernet ports, so one can go to the robot and one to the plant, and 32 GB of eMMC, so no SD card is needed. It is rated -25 to +60 °C. The datasheet power budget is **22 W "incl. 2 x 900 mA USB load"**, so each USB port is budgeted for 900 mA, which covers a D435. It is built on the Raspberry Pi CM5 (BCM2712, 4x Cortex-A76 at 2.4 GHz), the same SoC as a Pi 5. Sources: [product page](https://revolutionpi.com/en/revpi-connect-5), [datasheet PDF](https://revolutionpi.com/fileadmin/downloads/datasheets/Datasheet_RevPi-Connect-5.pdf).
- **OS.** Flash Debian arm64 on it (decided 2026-09-28), not the shipped RevPi OS; `deploy/pi/` targets Debian.
- **Variant chosen.** 100416 = 8 GB RAM, 32 GB eMMC, RS485, no Wi-Fi. The 4 GB variant (100412) costs $81 less at Phytools. It has not been tested with the cockpit, so I did not pick it.
- **Manufacturer price.** The [KUNBUS price overview](https://revolutionpi.com/en/ordering/overview-products-and-prices) lists 100416 at EUR 579.00 base, plus a **EUR 70.00 RAM surcharge** and a **EUR 10.00 eMMC surcharge**. That is EUR 659.00 net (about $749.81), seen 2026-09-28. The [shop page](https://revolutionpi.com/shop/en/revpi-connect-5) shows EUR 581.00 net / EUR 691.39 gross for the entry model, which is EUR 536 + 35 + 10. Shop lead time shown: "5 - 8 working days".
- **Distributor price (used in the total).** [Phytools (US)](https://phytools.com/products/revpi-connect-5): KU-PR100416 = **$796.00**, in stock, 2026-09-28. For comparison, KU-PR100412 (4 GB) = $715.00. Other distributors: [Digi-Key product highlight](https://www.digikey.com/en/product-highlight/k/kunbus/revpi-connect-5), "variants range from $733.72 to $966.68" (2026-09-28). [Mouser listing](https://www.mouser.com/en/new/kunbus/kunbus-revpi-connect-5): price not found (the page timed out). [RS UK 0428438](https://uk.rs-online.com/web/p/industrial-computers/0428438): the fetcher got a 403. A search snippet showed GBP 646.61 for the 32 GB / 8 GB model, unverified.
- **Price history.**
  - Launched 2024-11-27 with "very limited availability at product launch" ([KUNBUS forum announcement](https://revolutionpi.com/forum/viewtopic.php?t=4575)).
  - 2025-05-21: shop entry price EUR 499.00 net / EUR 593.81 gross ([Wayback snapshot](https://web.archive.org/web/20250521171540/https://revolutionpi.com/shop/en/revpi-connect-5)).
  - By 2026-06-09: base list EUR 536.00 for 100412, before the RAM surcharge ([Wayback snapshot](https://web.archive.org/web/20260609002435/https://revolutionpi.com/en/ordering/overview-products-and-prices)).
  - 2026-09-28: EUR 536 base + EUR 35 RAM + EUR 10 eMMC surcharge = EUR 581 for the entry model.
  - That is **+16 % on the entry model in 16 months**, driven by the separately itemised RAM surcharge (see §4, memory prices).

### K1-alt2: Industrial PC (alternative): CompuLab IOT-GATE-RPI5

- **Why it's second.** It is built on the CM5, takes **12 to 24 V DC** with reverse-polarity protection, and supports DIN-rail or wall mounting. It is rated 0 to 60 °C (commercial) or -40 to 80 °C (industrial). But it has only **one USB 3.0 Type-A** port (plus one USB 2.0), which is enough for one D435 and leaves no spare. Its Ethernet is one GbE plus one **100 Mbit/s** port. Its eMMC is 16 to 64 GB, with an optional NVMe. Specs: [product page](https://www.compulab.com/products/iot-gateways/iot-gate-rpi5-industrial-raspberry-pi-iot-edge-gateway/), [LinuxGizmos, 2026-05-08](https://linuxgizmos.com/iot-gate-rpi5-is-a-fanless-raspberry-pi-cm5-gateway-with-rs485-and-can-fd/).
- **Part numbers and prices** (CompuLab webshop, 2026-09-28):
  - IOTG-RPI5-D8N64-WB-JT910G-SIO-XL (8 GB / 64 GB eMMC, with an LTE modem this kit does not need): **$485.00**, "in stock, ships within 2 business days" ([shop](https://shop.compulab.com/product/iot-gate-rpi5-8gb-ram-64gb-emmc-lte/)).
  - IOTG-RPI5-D4N32-WB-JT910G-SIO-XL (4 GB / 32 GB): $393.00 ([shop](https://shop.compulab.com/product/iot-gate-rpi5-4gb-ram-32gb-emmc-lte/)).
  - IOTG2-ACC-DINCLP DIN-rail clip: **$2.50** ([shop](https://shop.compulab.com/product/iotg2-din-rail-clip/)).
  - Online orders are capped at 5 units per customer.
  - The webshop lists only the LTE configurations. A no-modem SKU may exist through sales (not verified).
- **Distributor:** not found. CompuLab sells direct.
- **Price history:** not found. The product is new: announced around May 2026.

### Candidates screened and rejected (no line in the kit)

The D435 needs a real USB 3 link. At 848x480 @ 30 fps, depth (Z16, 2 B/px) plus colour (YUYV, 2 B/px) is about 2 x 195 = **about 390 Mbit/s of payload**, which is above what USB 2.0 carries in practice. The repo also measured that a D435 on USB 2 **offers no 848x480 colour mode at all** (`docs/realsense.md`, *USB 2 link*). A USB-2-only box cannot run the default pipeline.

| Candidate | Compute | USB to camera | Verdict | Price seen |
|-----------|---------|---------------|---------|-----------|
| Raspberry Pi 5 8 GB + DIN case | BCM2712 | 2x USB 3.0, 5 Gbit/s simultaneous ([RPi](https://www.raspberrypi.com/products/raspberry-pi-5/)) | USB is fine. Rejected as the *kit* PC for three reasons. It needs 5 V / 5 A **USB-C PD**, so a 24 V cabinet needs a PD converter. On a 3 A supply the USB ports are limited to **600 mA total** (1.6 A only after a 5 A PD negotiation; [RPi docs](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc)). It boots from microSD. And at $175 the board alone has lost most of its price advantage. | SC1432: **$175** official (see history below); Digi-Key [SC1432](https://www.digikey.com/en/products/detail/raspberry-pi/SC1432/21658257) $175.00 (search snippet; page 403); [Olimex](https://www.olimex.com/Products/RaspberryPi/RPi5-8GB/) EUR 180.60 (about $205.49), **out of stock**, 2026-09-29 (Nick asked about this listing: it is the bare official board, so the same PD / 600 mA / microSD objections apply, and it is dearer than the Pi 4 line) |
| Raspberry Pi 4 Model B 4 GB + DIN case | BCM2711 | 2x USB 3.0 | **Chosen 2026-09-29 (K1).** Slower than a Pi 5 but enough; 1.2 A of USB on a 3 A supply. | $120.00 Adafruit, in stock; official $95 (see K1) |
| KUNBUS RevPi Connect 4 | CM4 (BCM2711, Cortex-A72 at 1.5 GHz) | 2x USB 3.2 Gen 1, 800 mA per port at 24 V ([datasheet](https://revolutionpi.com/fileadmin/downloads/datasheets/Technical_Datasheet_RevPi_Connect-4.pdf): 10.8 to 28.8 V, 20 W max, -25 to +55 °C) | The industrial packaging of the Pi 4 line. Viable, but at Phytools it costs $78 less than the Connect 5 for a slower SoC, so if you want a cabinet PC buy the Connect 5 (K1-alt). | Phytools [KU-PR100378](https://phytools.com/products/revpi-connect-4) (4 GB / 32 GB) $637.00, KU-PR100376 (2 GB / 8 GB) $544.00; Mouser PR100378 $624.40 (search snippets, 2026-09-29). KUNBUS list: 100378 EUR 493.00 + EUR 35 RAM + EUR 10 eMMC ([price overview](https://revolutionpi.com/en/ordering/overview-products-and-prices)) |
| OnLogic Factor 201 (FR201) | CM4 | 1x USB 3.2 Gen 1 + 2x USB 2.0 | Would stream one D435, but the CM4's Cortex-A72 is the slower SoC. Not picked. | $333 at launch, 2022-03-15 ([LinuxGizmos](https://linuxgizmos.com/raspberry-pi-cm4-powered-gateway-with-dual-m-2-b-key-slots-starts-at-333/)). Current: not verified ([store](https://www.onlogic.com/store/fr201/) did not render; a search snippet said $744.50). |
| OnLogic Factor 202 (FR202) | CM4 | 1x USB 3.2 Gen 1 + 2x USB 2.0 ([OnLogic docs](https://support.onlogic.com/product-documentation/industrial-products/factor-fr200-series/fr202)) | Same as FR201, plus I/O and a higher price. | From $887 (4 GB, 64 GB SSD), 2022-11-18 ([LinuxGizmos](https://linuxgizmos.com/onlogic-expands-raspberry-pi-powered-industrial-computing-devices/)). Current: not found. |
| Seeed reComputer R1000 | CM4 | **USB 2.0 only** (2x USB-A 2.0; [Seeed wiki](https://wiki.seeedstudio.com/recomputer_r/)) | **Cannot stream 848x480 depth + colour.** Rejected. | "Expected to launch at $209" (4 GB / 32 GB), per a search snippet of [CNX Software 2024-10-09](https://www.cnx-software.com/2024/10/09/recomputer-r1000-is-a-raspberry-pi-cm4-powered-iot-gateway-for-edge-ai-applications/); unverified |
| Seeed reTerminal DM | CM4 | 2x USB 2.0 standard; 2x USB 3.0 **optional** ([CNX 2023-06-07](https://www.cnx-software.com/2023/06/07/reterminal-dm-a-raspberry-pi-cm4-powered-10-1-inch-hmi-controller/)) | Only the USB 3 option works. It is a 10.1 in HMI, which is more than the kit needs. Rejected. | $409 (samples, 2023-06) |
| Seeed reComputer Industrial R20xx | CM5 | 3x USB-A 3.0 + 1x USB 2.0, 9 to 36 V DC ([Seeed wiki](https://wiki.seeedstudio.com/recomputer_industrial_r20xx_getting_start/)) | Technically viable. It is a strong third choice, but I found no price. | not found |
| EDATEC ED-IPC3020 | Pi 5 board | 2x USB 3.0 | Viable USB, but it is powered through USB-C like a bare Pi 5. | $165 (4 GB) / $207 (8 GB) at launch, per [Liliputing](https://liliputing.com/edatec-ed-ipc3020-is-a-raspberry-pi-5-powered-industrial-pc-with-nvme-and-analog-audio-support/) (search snippet). Current: not found. |

**Raspberry Pi 5 8 GB price history** (the same memory pressure drives CM5 and every CM5 box above):

| Date | Price | Change | Source |
|------|-------|--------|--------|
| 2023 launch | $80 | — | [RPi news 2025-12-01](https://www.raspberrypi.com/news/1gb-raspberry-pi-5-now-available-at-45-and-memory-driven-price-rises/) (the "old price" column); Digi-Key $80 in Nov 2023 ([Slickdeals](https://slickdeals.net/f/17093713-raspberry-pi-5-8gb-in-stock-80-shipping-digi-key)) |
| 2025-10-01 | $80 | CM4/CM5 8 GB +$10; Pi 5 not listed | [RPi news](https://www.raspberrypi.com/news/5-10-price-increases-for-some-4gb-and-8gb-products/) |
| 2025-12-01 | $95 | +$15 | [RPi news](https://www.raspberrypi.com/news/1gb-raspberry-pi-5-now-available-at-45-and-memory-driven-price-rises/) |
| 2026-02-02 | $125 | +$30 (8 GB density) | [RPi news](https://www.raspberrypi.com/news/more-memory-driven-price-rises/) |
| 2026-04-01 | **$175** | +$50 (Pi 4/5 8 GB; CM5 8 GB also +$50) | [RPi news](https://www.raspberrypi.com/news/a-new-3gb-raspberry-pi-4-for-83-75-and-more-memory-driven-price-increases/) |

The $175 figure is the sum of Raspberry Pi's own announcements. The official product page did not render the 8 GB price for the fetcher (it did show 16 GB = $305). [How-To Geek (2026-08-06)](https://www.howtogeek.com/buying-raspberry-pi-2026-better-have-a-very-good-reason/) quotes $180 for 8 GB, which may be a reseller price.

### K2: DIN-rail PSU: Mean Well HDR-30-5

- 5 V, 3 A, 15 W, DIN rail (TS35), adjustable output. 3 A is exactly the Pi 4's stated minimum ("5V DC via USB-C connector (minimum 3A)"), and the budget fits: 600 mA typical for the board plus at most 1.2 A on USB. Trim it to 5.1 V. Feed the Pi's USB-C socket through a screw-terminal-to-USB-C pigtail (a few dollars, not priced); the Pi 4 spec also allows "5V DC via GPIO header (minimum 3A)" if you would rather land wires on the header. No USB-PD is involved, so there is nothing to negotiate.
- **Price:** [TRC Electronics](https://www.trcelectronics.com/products/mean-well-hdr-30-5) **$19.73, 26 in stock**, 2026-09-29. Also Jameco $15.40, RS $18.97, Arrow $23.69 (search snippets, 2026-09-29); Digi-Key [HDR-30-5](https://www.digikey.com/en/products/detail/mean-well-usa-inc/HDR-30-5/7703801) 146 in stock, price not fetched (403).
- History: not found.

### K2-alt: DIN-rail PSU for the industrial PCs: Mean Well HDR-60-24

- 24 V, 2.5 A, 60 W, DIN rail. This gives headroom over the RevPi's 22 W maximum (the CompuLab takes 12 to 24 V too). Use a dedicated supply rather than borrowing the UR controller's I/O 24 V. If you do use the controller's supply, check its current budget in the UR manual first.
- Current: **Digi-Key [HDR-60-24](https://www.digikey.com/en/products/detail/mean-well-usa-inc/HDR-60-24/7703804) $20.10.** The live page returns 403 to the fetcher; $20.10 is from the search listing on 2026-09-28 and matches the 2026-03-28 Wayback snapshot. Also [PowerSupplyMall](https://powersupplymall.com/products/mean-well-hdr-60-24-ultra-slim-step-shape-power-supply-60w-24v-din-rail) $24.34, in stock, 2026-09-28.
- History (Digi-Key via Wayback): **$25.30** on [2024-07-20](https://web.archive.org/web/20240720163629/https://www.digikey.com/en/products/detail/mean-well-usa-inc/HDR-60-24/7703804), then $20.10 on [2025-11-07](https://web.archive.org/web/20251107191418/https://www.digikey.com/en/products/detail/mean-well-usa-inc/HDR-60-24/7703804) and [2026-03-28](https://web.archive.org/web/20260328062616/https://www.digikey.com/en/products/detail/mean-well-usa-inc/HDR-60-24/7703804).
- It also needs a length of 35 mm DIN rail and a mains cord or terminal wiring, which the cabinet usually supplies. Not priced.

### K3: Depth camera: RealSense D435 (what the code and the bracket are built for)

- **Part number** 82635AWGDVKPRQ. **Manufacturer:** RealSense Inc. (spun out of Intel on 2025-07-11). The camera has a USB-C 3.1 Gen 1 port with screw-lock threads, a 1/4-20 and two M3 mounting points, and measures 90 x 25 x 25 mm ([product page](https://www.realsenseai.com/products/stereo-depth-camera-d435/)). Launched Q1 2018 ([Intel ARK](https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html)).
- **Current price.** [RealSense store](https://store.realsenseai.com/buy-intel-realsense-depth-camera-d435.html): **$314.00**, "**Out of Stock**", lead time "**6-8 weeks**". A tariff surcharge applies to store orders from 2026-02-03. All seen 2026-09-28. The store page does not state the included cable's length.
- **Distributors.** [Digi-Key 82635AWGDVKPRQ](https://www.digikey.com/en/products/detail/intel-realsense/82635AWGDVKPRQ/9926002): $333.75, no stock ("request stock notification"). This is from a search snippet; the page returns 403. [Mouser](https://www.mouser.com/ProductDetail/Intel/82635AWGDVKPRQ?qs=wd5RIQLrsJh1kH/NlgqopQ%3D%3D): price not found (timed out). [B&H](https://www.bhphotovideo.com/c/product/1432415-REG/intel_82635awgdvkprq_realsense_d435_webcam.html): not found (403). Broker listings with "thousands in stock" (Kynix and others) are not recommended sources.
- **Price history** (the Intel store page via Wayback; the store moved to store.realsenseai.com in 2025):

| Date seen | Price | Source |
|-----------|-------|--------|
| 2018-01-18 launch | $179 (D415 $149) | [SlashGear](https://www.slashgear.com/intel-realsense-d415-d435-depth-cameras-launch-with-d4-vision-processor-18516337/) |
| 2019-05-20 to 2021-01-17 | $179 | [Wayback 2019-05-20](https://web.archive.org/web/20190520015640/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html), [2021-01-17](https://web.archive.org/web/20210117143826/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html) |
| 2021-06-01 | $189 | [Wayback](https://web.archive.org/web/20210601164220/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html) |
| 2021-12-04 to 2022-05-19 | $299 | [Wayback 2021-12-04](https://web.archive.org/web/20211204033605/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html), [2022-05-19](https://web.archive.org/web/20220519201018/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html) |
| 2022-10-06 to 2025-02-09 | $314 | [Wayback 2022-10-06](https://web.archive.org/web/20221006030142/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html), [2025-02-09](https://web.archive.org/web/20250209230223/https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html) |
| 2025-08-30 (after the spin-out) | $314 | [Wayback, store.realsenseai.com](https://web.archive.org/web/20250830083619/https://store.realsenseai.com/buy-intel-realsense-depth-camera-d435.html) |
| 2026-09-28 | $314 + tariff surcharge, out of stock, 6 to 8 weeks | live store |

[camelcamelcamel B07BLS5477](https://camelcamelcamel.com/product/B07BLS5477) (the Amazon listing) blocks the fetcher, so the Amazon history is not found.

- **D435 vs D435i vs D405** ([store list prices, 2026-09-28](https://store.realsenseai.com/)):
  - **D435 ($314)** is what is used and what `bracket.py` locates by its tripod boss and body outline (Rev C).
  - **D435i ($334)** is the same body with an IMU. The pick pipeline does not use the IMU, so the $20 buys nothing here. The bracket fits it, since the body and mounting points are the same.
  - **D436 ($354)** is listed as a D435i with a 1 MP global-shutter, wider-FOV RGB sensor ([librealsense #14858](https://github.com/realsenseai/librealsense/issues/14858)). Not evaluated.
  - **D405 ($272)** is short range (7 to 50 cm), has no IR projector, takes its RGB from the depth imagers, and measures 42 x 42 x 23 mm ([Robot Report](https://www.therobotreport.com/intel-adds-short-range-realsense-d405-depth-camera/)). It would remove the D435's under-0.2 m blind zone that forces the pick routine's 0.24 m close look. It needs a new bracket and has not been tested with this code. Optional line O5.

### K4: USB cable: Newnex U3HLA01C12-030 (high-flex, A to C, dual screw-lock, 3 m)

- **Why.** It is rated for **6,000,000 flex cycles**, has a TPE jacket, and has dual screw-lock USB-C plugs that match the D435's locking threads. Newnex rates the 3 m length at 5 Gbit/s, 5 V / 3 A, "max travel 2 m" (the length that may flex). Newnex's RealSense page says its A-to-C cables are made "for the Intel RealSense D435, D455". Sources: [high-flex family](https://newnex.com/usb-3-high-flex-cables.php), [A-to-C family](https://newnex.com/usb-type-c-cables-legacy.php), [RealSense page](https://newnex.com/realsense-3d-camera-connectivity.php), [drawing PDF](https://newnex.com/documents/U3HLA01C12-030.PDF).
- **Current price.** [NTC Distributing](https://www.ntcdistributing.com/usb-c/high-flex-usb-3-2-a-to-c-cable-with-dual-screw-locking-1m-2m-3-m-5-m/?sku=U3HLA01C12-030) (the distributor Newnex links to): **$188.00** (3 m). Same page: $148.00 for 1 m and $228.00 for 5 m. [Industrial Component](https://www.industrialcomponent.com/newnex/u3hla01c12/): $149.00 for 1 m; other lengths not shown. All 2026-09-28. Digi-Key and Mouser: not found.
- **History:** not found.
- **Length choice and a conflict with the repo's own note.** `docs/realsense.md` (Troubleshooting) says "≤ 2 m active-free cable, direct port". Newnex rates this 3 m passive cable for 5 Gbit/s, but that is the vendor's rating, not a test on this camera. **Verify at SuperSpeed on the first build** (`rs-enumerate-devices` / the doctor's USB line) before shipping 3 m. If it drops to USB 2, or the run is longer than 3 m, use the active 5 m cable (O2: [ULHLU11AC12-5M](https://newnex.com/high-flex-active-usb-3-cable.php), 4.7 m travel, minimum bend radius 12 x OD, about 75 mm; $399.00 at [Industrial Component](https://industrialcomponent.com/newnex/), 2026-09-28).
- **igus chainflex:** igus lists USB 3.0 chainflex cable (CFBUS.PUR.068) and harnessed readycables with USB-A / micro-B ends ([example](https://www.igus.com/product/readycable_buscable_USB9540201)). I found **no stocked igus USB-C screw-lock assembly**, so it is not a line here.

### K5: Ethernet: L-com TRD695SCR-BLK-10

- Shielded Cat6 (26 AWG stranded STP), RJ45 to RJ45, PVC jacket, 10 ft. It runs PC to controller inside or next to the cabinet, not along the arm, so a high-flex cable is not needed.
- Current: **[L-com](https://www.l-com.com/ethernet-shielded-cat-6-cable-rj45-rj45-pvc-jacket-black-100-ft) $42.19** at qty 1 (quantity breaks shown at $40.08, $37.97 and $36.28), 2026-09-28. Distributors carrying TRD695SCR-10 (grey): [Newark 84M7466](https://www.newark.com/l-com/trd695scr-10/patch-cord-category-6-shielded/dp/84M7466), [RS US 70126903](https://us.rs-online.com/product/l-com/trd695scr-10/70126903/). Prices not found (not fetched or blocked).
- If the cable has to ride a moving axis, use L-com's high-flex TRD695AHF series instead: 5 ft = $111.99 at qty 1 ([L-com](https://www.l-com.com/ethernet-double-shielded-cat6a-outdoor-industrial-high-flex-ethernet-cable-teal-rj45-rj45-50ft)).
- History: not found.

### K6: Cable management along the arm

- The Hand-E needs no extra cable: it runs through the tool connector. Only the USB cable has to follow the arm. It leaves the camera's end face and runs down with the tool-I/O side (bracket README §6 A1), so hook-and-loop straps at each link are enough for a UR3e.
- **VELCRO Brand ONE-WRAP, 1/2 in x 25 yd, black, part 189755:** **$50.00** for 1 roll at [iTapeStore](https://www.itapestore.com/velcro-brand-one-wrap-straps-one-half-inch-by-25-yard-rolls) ($39.90 each for 2 to 5 rolls), 2026-09-28. One roll covers many cells; the per-cell use is a metre or two. History: not found.
- **Upgrade (O1): igus triflex R "Universal Robots kit" for UR3(e)**, [TRE.918.067.LAE30](https://www.igus.eu/product/20680). The kit contains 1 mounting bracket with strain relief, the cobot clamps, and TRE.30 e-chain. It costs **EUR 241.15** (about $274.38) on igus.eu, "ready to ship within 24 hours", 2026-09-28. The size-40 version, TRE.918.067.LAE40, is EUR 247.65. The US price was not checked. History: not found.
- Spiral wrap (for example [Alpha Wire SW3](https://www.alphawire.com/products/accessories/fit-wire-management/spiral%20wrap/sw3)) is an alternative. Price not verified.

### K7: Bracket print material: PPA-CF

- The bracket README (§5) specifies PPA-CF (Bambu PPA-CF or Polymaker Fiberon PPA-CF). The e-Series Rev C print is estimated at about 36 g of part and about 40 g of filament (Rev B.2: 27 g / 31 g), printed on a hardened 0.4 mm nozzle at 300 to 320 °C in an enclosure. Alternatives: PA6-CF or PET-CF.
- **Bambu Lab PPA-CF, black, 0.75 kg with spool, SKU N06-K0-1.75-750-SPL:** **$149.99**, available, from the [Bambu US store product JSON](https://bambulab-us.myshopify.com/products/ppa-cf) (2026-09-28). That is $0.20/g, so 40 g = **$8.00** per bracket. Distributor: [MatterHackers](https://www.matterhackers.com/store/l/bambu-lab-ppa-cf-filament/sk/MF7C2ZUL), $198.00 per 0.75 kg (search snippet). History: not found.
- **Polymaker Fiberon PPA-CF:** I found no such product in Polymaker's shop on 2026-09-28. The nearest Fiberon grade is [Fiberon PA6-CF20](https://shop.polymaker.com/products/fiberon-pa6-cf20) (FG03001): $39.99 per 0.5 kg, in stock, with 3 kg for $159.99. The README's "Polymaker Fiberon PPA-CF" should be checked.
- **Heat-set inserts:** **none.** The bracket README's hardware list (§4) has no inserts. The one camera screw goes into the D435's own 1/4-20 thread (Rev C: the lip locates the camera, the M3s are unused), and the tool bolts pass straight through the plate into the flange.
- The UR20/UR30 Rev C print is estimated at about 60 g of filament (about $12). It is not part of the UR3e kit.

### K8 to K12: Fasteners (bracket README §4)

- **K8. 1/4-20 UNC x 3/8 in flat head socket cap, 82 deg, 18-8 stainless, qty 1 (Rev C, 2026-10-02):** the Rev C wall is thinner, so the 1/2 in screw would go ≈ 8.9 mm into the camera; 3/8 in gives ≈ 5.7 mm. The 1/2 in version was [U-Turn Fasteners SCFH025C0050SS](https://www.uturnfasteners.com/1-4-20-x-1-2-flat-head-socket-cap-screw-18-8-stainless-steel/), **$0.08**, 255 in stock, 2026-09-28; the 3/8 in part number and price are **not checked**. The README says to measure the D435's thread depth first (A4); a 5/16 in screw is the fallback. Torque 1.5 N·m. History: not found.
- **K9. Removed in bracket Rev C (2026-10-02):** the two M3 x 8 flat heads ([Monster Bolts](https://monsterbolts.com/products/mach-phil-flat-a2-m3), $0.89 / 10) are no longer used; a chamfered lip locates the camera.
- **K10. Tool bolts, qty 4.** The README rule is "the tool's own bolts, 8 mm longer". The Hand-E e-Series coupling (GRP-ES-CPL-062) mounts with M6 x 10 low-head socket cap screws: the manual lists M6-1.0 low-head clearance on the ISO 50-4-M6 coupling and M6 x 10 screws for coupling mounting ([Hand-E e-Series manual PDF](https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Instruction_Manual_e-Series_PDF_20190122.pdf), §3.4.2 and §6). Two things need checking before the first fit:
  - "+8 mm" gives M6 x 18, but the adapter plate is only **6 mm** thick, so M6 x 18 adds 2 mm of engagement in the flange. The flange allows **≤ 8 mm** of engagement (README §2).
  - Monster Bolts stocks DIN 7984 A2 M6 in 12, 16 and 20 mm, not 18.
  - This BOM lists **M6 x 16** (10 + 6 = the Hand-E's original engagement). [Monster Bolts](https://monsterbolts.com/products/socket-low-hd-a2-m6) 10-pack "SA2 M06 x 016.0010" costs **$3.05** (the 20 mm 10-pack is $3.55), 2026-09-28.
  - **Measure the Hand-E coupling's actual screw engagement before ordering.** The M6 x 10 length is from the dual-gripper mounting step in Robotiq's manual; I did not find it stated for the single-gripper wrist mount. History: not found.
- **K11. Dowel pin, dia. 6 m6 x 20, qty 1.** It passes through the adapter's pin slot into the flange and stands proud for the tool's pin hole. Robotiq's manual says robot-side hardware is not supplied: "Unless specified, screws, dowel pins and other hardware are included only for the Gripper side, never for the robot side." Source for the ISO 8734 m6 specification: [Fuller Fasteners](https://fullerfasteners.com/products/iso-8734-hardened-dowel-pins-m6/). **Price: not found.** McMaster-Carr and MISUMI (both carry m6-tolerance 6 x 20 pins: [McMaster](https://www.mcmaster.com/products/dowel-pins/thread-size~m6-2/), [MISUMI](https://us.misumi-ec.com/vona2/press/P0300000000/P0302000000/P0302010000/P0302010100/?CategorySpec=00000031438::c)) blocked the fetcher.
- **K12. Medium-strength threadlocker** on the flat heads (bracket README §4). Robotiq also calls for it on the gripper screws. **Price: not found.**

### C1: Universal Robots UR3e (customer-supplied, reference)

- UR does not publish list prices. The only priced listing I could verify is [Devonics](https://www.devonics.com/product-page/universal-robots-ur3e): "$30,852.00 USD per unit" base, shown as $33,011.00 with options, 2026-09-28. Aggregators put the UR3e at $23,000 to $38,000 for 2026 ([cobotcost.com](https://cobotcost.com/vendor/ur3e/), [Standard Bots](https://standardbots.com/blog/universal-robot-price)). Those are ranges, not quotes.
- History: not found. The e-Series launched in 2018.

### C2: Robotiq Hand-E, e-Series kit (customer-supplied, reference)

- **HND-ES-UR-KIT** contains the gripper unit HND-GRP-001, fingertip starting kit HND-TIP-START-KIT, coupling GRP-ES-CPL-062, screw kit, and a URCap USB stick ([Robotiq manual](https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Instruction_Manual_e-Series_PDF_20190122.pdf)).
- Current: **[Automation Distribution](https://automationdistribution.com/robotiq-hnd-es-ur-kit-ur-kit-hand-e-for-e-series/) $8,416.41**, stock not shown, 2026-09-28. Other listings: [RS US 72506890](https://us.rs-online.com/product/robotiq/hnd-es-ur-kit/72506890/) (403) and [Logic Control](https://www.logic-control.com/robotiq-hnd-es-ur-kit), prices not found.
- History: not found.

---

## 3. Subtotals

| Group | Amount |
|-------|--------|
| **Kit per cell, primary PC (Raspberry Pi 4 4 GB + case + microSD + 5 V PSU)** | **$785.39** + K8, K11, K12 (not found) |
| Kit per cell, industrial PC (RevPi Connect 5, 100416 + HDR-60-24) | $1,421.34 + K8, K11, K12 |
| Kit per cell, industrial PC (CompuLab IOT-GATE-RPI5 8 GB + DIN clip + HDR-60-24) | $1,112.84 + K8, K11, K12 |
| Kit per cell, RevPi bought from KUNBUS direct instead of Phytools (EUR 659 = $749.81) | $1,375.15 + K8, K11, K12 + import duty/shipping |
| Customer-supplied (UR3e + Hand-E kit) | $39,268.41 |
| Optional upgrades (O1 igus dress pack + O2 active 5 m cable, if both) | about $673.38 |

Recurring consumables are counted per cell: the ONE-WRAP roll ($50.00) and the PPA-CF spool ($149.99, about 18 Rev C brackets per spool) are bought once and shared across builds. Costing only what one cell uses, the primary kit is about **$740** (hook-and-loop about $4, filament $8.00); the RevPi kit about $1,375.

---

## 4. Lead times and availability risks

1. **RealSense D435: out of stock at the source, and a change of owner.**
   - The RealSense store shows the D435 as "Out of Stock", lead time **6 to 8 weeks**, with a **tariff surcharge on store orders since 2026-02-03** (2026-09-28). Digi-Key shows no stock (search snippet).
   - RealSense spun out of Intel on **2025-07-11** with a $50 M Series A ([RealSense press release](https://www.realsenseai.com/news-insights/news/realsense-completes-spin-out-from-intel-raises-50-million-to-accelerate-ai-powered-vision-for-robotics-and-biometrics/)).
   - On **2026-09-22** Cognex agreed to buy RealSense for about **$500 M in cash**, **expected to close in Q4 2026** ([Cognex 8-K exhibit 99.1](https://www.sec.gov/Archives/edgar/data/851205/000085120526000071/exhibit991-pressrelease.htm)). The release says nothing about D400 continuity or pricing.
   - Plan on **8+ weeks for the camera**, buy cameras ahead of kit builds, and re-check the list price after the deal closes.
   - The D435's list price has already gone up 75 % since launch ($179 to $314, all of it between mid-2021 and late 2022).
2. **Memory-driven price rises on everything Raspberry Pi, the Pi 4 included.**
   - The Pi 4 4 GB went $55 to $70 (2026-02-02) to $95 (2026-04-01); resellers ask $100 to $120. It is a 2019 board; every rise so far has hit it too.
   - Raspberry Pi raised prices four times between 2025-10-01 and 2026-04-01. The Pi 5 8 GB went from $80 to $175, and the CM5 8 GB rose by +$10 in Oct 2025, +$30 in Feb 2026 (the per-density table) and +$50 in Apr 2026.
   - KUNBUS now itemises a **RAM surcharge (EUR 35 for 4 GB, EUR 70 for 8 GB)** and an eMMC surcharge (EUR 10) on top of the RevPi list price. Expect the PC line to move again.
   - Raspberry Pi says it will reverse the rises "once memory prices return to their long-term downward trajectory" ([RPi, 2026-02-02](https://www.raspberrypi.com/news/more-memory-driven-price-rises/)). No date given.
3. **RevPi Connect 5:** shipped with "very limited availability" at launch (Nov 2024). It is now "5 - 8 working days" from KUNBUS and in stock at Phytools (2026-09-28).
4. **CompuLab IOT-GATE-RPI5:** in stock with 2-day shipping, but **webshop orders are capped at 5 units per customer**. The listed SKUs carry an unneeded LTE modem.
5. **USB cable:** a single-source Newnex part (NTC Distributing / Industrial Component). No lead time is shown.
6. **Hand-E / UR3e:** priced through integrators; no public list price or lead time.

---

## 5. Lines I could not verify

- K11 dowel pin (6 m6 x 20) price
- K12 medium threadlocker price
- Price history for: RevPi (before May 2025), CompuLab, the Newnex cable, the L-com cable, the Bambu PPA-CF, fasteners, ONE-WRAP, igus, UR3e, Hand-E
- The D435's Amazon (camelcamelcamel) history: blocked
- Current prices at Digi-Key, Mouser, RS, B&H and McMaster: those pages block the fetcher. The Digi-Key numbers above are search-listing snippets or Wayback snapshots.
- Seeed reComputer R20xx price; OnLogic FR201/FR202 current prices
- Micro Center's $94.99 for the Pi 4 4 GB (403); Mouser's $30.43 for the industrial microSD (page timed out); the Pi 4's stock at PiShop.us
- Per-frame timing of the cockpit on a Pi 4 vs a Pi 5 (nothing measured on either)
- Whether the D435 box cable length is stated anywhere current (the store page does not)
- The Hand-E coupling's single-mount M6 screw length (K10 note)
- Polymaker "Fiberon PPA-CF": not found as a product
