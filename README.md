# Aktīvās kontroles raķetes avionikas sistēma

## Projekta apraksts
Šis projekts paredz aktīvās kontroles raķetes avionikas sistēmas izstrādi, kas ietver gan raķetē iebūvētu avionikas datoru, gan zemes bāzi datu saņemšanai, uzglabāšanai un analīzei. Sistēma nodrošina aktīvu trajektorijas korekciju reāllaikā, izmantojot sensoru datus, vadības algoritmus un elektroniskos izpildmehānismus.

Projektā tiek apvienotas iegultās sistēmas, signālu apstrāde, vadības algoritmi, telemetrija un lietotāja saskarnes izstrāde.

**Projektu izstrādā:**  
Aleksandrs Jesiļevskis  
Miks Balodis  

---

## Projekta mērķis
Izstrādāt:
- raķetes avionikas datoru ar aktīvas trajektorijas korekcijas iespējām,
- zemes bāzi lidojuma datu saņemšanai un analīzei,
- drošu un papildināmu sistēmu, kas piemērota eksperimentālām lieljaudas raķetēm.

Sistēmai jāspēj:
- reāllaikā noteikt raķetes stāvokli,
- veikt aktīvo vadību ar vairākām kontroles spurām,
- ierakstīt un uzglabāt lidojuma datus,
- nodrošināt saziņu ar zemes bāzi lidojuma laikā.

---

## Idejas rašanās
Ideja radās, balstoties uz **RTU Lieljaudas raķeškomandas (RTU HPR)** 2025. gada 1. semestra projektu – aktīvās rotācijas kontroles raķeti. Šī projekta ietvaros tika izmantots RP2040 mikrokontrolieris, kuram konstatēti vairāki ierobežojumi:
- nav iebūvēta peldošā komata vienības (FPU),
- ierobežota programmas atmiņa,
- augsts enerģijas patēriņš.

Lai šos trūkumus novērstu, projektā tiek izvēlēts **STM32F405RGT6**, kas nodrošina augstāku veiktspēju, FPU atbalstu un DSP instrukcijas.

---

## Idejas apraksts

### Projekta lietotāji
- studentu raķešu komandas,
- augstskolu/vidusskolu eksperimentēšanas grupas,
- iegulto sistēmu un vadības algoritmu izstrādātāji,
- eksperimentālu raķešu un bezpilota sistēmu entuziasti.

### Izmantotās tehnoloģijas
- **Iegultās sistēmas:** STM32 (ARM Cortex-M4)
- **Sensori:** barometrs, IMU, GNSS
- **Vadības algoritmi:** PID, Kalmana filtrs
- **Telemetrija:** sub-GHz radio sakari
- **Programmatūra:** C/C++, MatLab/Simulink (simulācijas), JavaScript un Svelte (GUI)
- **Projektēšana:** KiCad

### Piegādes formāts
Projekta rezultāts būs:
- raķetes avionikas sistēmas plate (PCB),
- iegultā programmatūra gatava demonstratīvam raķetes lidojumam (firmware),
- zemes bāzes programmatūra (GUI),
- dokumentācija.

Šāda pieeja ļauj sistēmu izmantot gan eksperimentālos lidojumos, gan simulācijās.

---

## Aktīvā kontrole
Aktīvā kontrole ir vadības sistēma, kas lidojuma laikā nepārtraukti:
- mēra raķetes stāvokli (leņķi, ātrumu, rotāciju),
- aprēķina nepieciešamos labojumus,
- ģenerē PWM signālus izpildmehānismiem servo kontrolētām spurām.

Sistēma atbalsta līdz **4 aktīvām spurām**, katrai paredzot atsevišķu PWM kanālu. Salīdzinājumā ar pasīvo kontroli, aktīvā kontrole spēj reaģēt uz ārējiem traucējumiem un uzlabot stabilitāti un precizitāti.

---

## Risinājuma arhitektūra

### Konceptuālais modelis
Sistēma sastāv no trim galvenajiem blokiem:
1. **Sensoru slānis** – datu iegūšana
2. **Vadības un apstrādes slānis** – aprēķini un lēmumu pieņemšana
3. **Izpildes un komunikācijas slānis** – kontrole, datu uzglabāšana un pārraide

### Tehniskā arhitektūra

#### MCU
- **STM32F405RGT6** - mikrokontrolieris datu apstrādei un komandu nodošanai

#### Sensori
- **BMP388** – barometrs augstuma noteikšanai
- **LSM6DSO** – IMU (akselerometrs + žiroskops)
- **MAX-M10S** – GNSS modulis augstas precizitātes pozīcijai

#### Datu uzglabāšana
- **MXL1606** – lidojuma datu ierakstīšanai (USB Mass Storage režīms)
- **24LC64 EEPROM** – konfigurācijas datiem

#### Telemetrija
- **EBYTE E22-400M22S** - LoRa moudlis saziņai ar zemes bāzi (433 MHz)

#### Vadība
- PWM signāli spuru servo
- PID regulators
- Kalmana filtrs stāvokļa novērtēšanai

---
## Izstrādes plāns (10 nedēļas)

Projekta izstrāde ir plānota 10 nedēļu laikā, pieņemot, ka avionikas plate jau ir uzdizainēta un pasūtīta, un nākamais solis ir tās salodēšana. Darbs tiek sadalīts starp diviem izstrādātājiem, lai maksimāli efektīvi izmantotu paralēlu izstrādi.

### Lomu sadalījums
- **Aleksandrs** – iegultās sistēmas programmēšana, STM32 firmware, sensoru draiveri, datu plūsma, vadība un telemetrija, sistēmas arhitektūras lēmumi,  integrācija, testēšana un dokumentācija.
- **Miks** – raķetes tehniskā daļa, mehāniskā integrācija, Matlab/Simulink simulācijas, vadības algoritmu modelēšana un zemes stacijas aparatūra, sistēmas arhitektūras lēmumi,  integrācija, testēšana un dokumentācija.

---

### 1. nedēļa – Sistēmas inicializācija un modeļu sagatavošana
**Aleksandrs:**
- STM32CubeIDE projekta izveide
- Pulksteņu (clock tree) un pamata perifēriju konfigurācija
- Debug interfeisa (SWD/UART) inicializācija

**Miks:**
- Raķetes kustības konceptuālā modeļa izstrāde
- Aktīvās kontroles uzdevuma definēšana
- Sensoru mērījumu teorētisko modeļu izveide

**Rezultāts:**  
MCU veiksmīgi startē un sistēmas matemātiskais modelis ir definēts.

---

### 2. nedēļa – Perifēriju un datu plūsmas pamati
**Aleksandrs:**
- GPIO, UART, SPI un I2C konfigurācija
- DMA izmantošanas pamati datu pārsūtīšanai
- Vienkārša datu izvada testēšana

**Miks:**
- PID regulatora struktūras izveide Matlab/Simulink
- Sākotnējo vadības parametru novērtēšana
- Avionikas plates salodēšana

**Rezultāts:**  
MCU spēj komunicēt ar perifērijām, un PID struktūra ir gatava simulācijām, plate ir salodēta.

---

### 3. nedēļa – Avionikas plates salodēšana un sensoru testēšana
**Aleksandrs:**
- IMU (LSM6DSO) un barometra (BMP388) draiveru izstrāde
- Sensoru datu nolasīšana un izvadīšana caur UART

**Miks:**
- Barošanas un signālu līmeņu pārbaude
- Sensoru fiziskās integrācijas novērtēšana

**Rezultāts:**  
Pilnībā funkcionējoši aktīvie sensori.

---

### 4. nedēļa – GNSS un sensoru apvienošana
**Aleksandrs:**
- GNSS (MAX-M10S) draivera izstrāde
- Laika sinhronizācija starp sensoriem
- Datu struktūru definēšana

**Miks:**
- Sensoru kalibrācijas metodes
- Kļūdu modelēšana
- Mērījumu precizitātes izvērtēšana

**Rezultāts:**  
Apvienoti visi sensoru dati vienotā laika skalā.

---

### 5. nedēļa – Aktīvā vadība un izpildmehānismi
**Aleksandrs:**
- PWM izvades konfigurācija
- Taimeru konfigurācija izpildmehānismiem
- Izpildmehānismu testēšana

**Miks:**
- PID parametru regulēšana Simulink vidē
- Stabilitātes analīze
- Vadības algoritma validācija simulācijā

**Rezultāts:**  
Izpildmehānismi reaģē uz vadības signāliem paredzētajā veidā.

---

### 6. nedēļa – Datu apstrāde un uzglabāšana
**Aleksandrs:**
- USB Mass Storage režīma ieviešana
- Datu struktūras optimizācija

**Miks:**
- Kalmana filtra izstrāde Simulink vidē
- Algoritma sagatavošana ieviešanai C kodā

**Rezultāts:**  
Stabili ierakstīti un izgūstami lidojuma dati.

---

### 7. nedēļa – Telemetrija un zemes stacijas aparatūra
**Aleksandrs:**
- Telemetrijas protokola izstrāde
- EBYTE E22 radio moduļa integrācija
- Datu pakešu formāta definēšana

**Miks:**
- Zemes stacijas plates arhitektūras izstrāde
- Komponentu izvēle un shēmas dizains
- Komunikācijas interfeisa plānošana

**Rezultāts:**  
Bezvadu datu pārraide starp raķeti un zemes staciju.

---

### 8. nedēļa – Zemes stacijas programmatūra un GUI
**Aleksandrs:**
- Seriālās komunikācijas protokols ar datoru
- Telemetrijas datu parsēšana

**Miks:**
- Grafiskās lietotāja saskarnes izstrāde (Svelte)
- Reāllaika grafiki un datu attēlošana
- Datu eksportēšana analīzei

**Rezultāts:**  
Darbojoša zemes stacija ar reāllaika datu vizualizāciju.

---

### 9. nedēļa – Integrācija un sistēmas testi
**Aleksandrs:**
- Pilna vadības cikla integrācija
- Kļūdu apstrāde un drošības mehānismi

**Miks:**
- Vibrāciju testi
- Sensoru trokšņu ietekmes pārbaude

**Rezultāts:**  
Stabila un integrēta sistēma bez kritiskām kļūdām.

---

### 10. nedēļa – Stabilizācija un dokumentācija
**Abi:**
- Koda un aparatūras kļūdu labošana
- Projekta dokumentācijas pabeigšana
- Demonstrācijas sagatavošana

**Rezultāts:**  
Pilnībā sagatavots projekts demonstrācijai un prezentācijai.

---

## Projekta izstrādes grafiks

![Projekta izstrādes Gantt diagramma](docs/gantt_diagramm.png)

---

## Licence
Licence tiks noteikta projekta vēlākā izstrādes posmā.

---

## Kontakti
Jautājumu vai sadarbības gadījumā sazināties ar projekta izstrādātājiem.
