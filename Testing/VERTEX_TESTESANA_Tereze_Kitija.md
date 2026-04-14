# VERTEX projekta testēšana

RTU Inženierzinātņu vidusskolas  
11. klases skolnieces  
**Kitija Kampiņa** un **Terēze Luīze Kīne**  

---

# Funkcionālie testi

| Testa veids | Ievade, darbība | Sagaidāmais rezultāts | Patiesais rezultāts |
|------------|-----------------|----------------------|---------------------|
| CSV failu ielāde un testu palaišana | CSV failu ielāde un testu palaišana | CSV faili tiek veiksmīgi ielādēti un tests tiek palaists. | Strādā. CSV faili ielādējas un testēšana sākas. |
| Testu atkārtošana | Jauna vai tā paša CSV faila ielāde un atkārtota testēšana | Iespējams ielādēt jaunu failu un atkārtoti palaist testu ar to pašu vai citu CSV. | Daļēji strādā. Jaunu failu var ielādēt, bet atkārtota tā paša faila testēšana nedarbojas. Noderētu "Refresh" funkcija. |
| Normāls pilns lidojums | Pilna lidojuma scenārija simulācija | Stāvokļi mainās secīgi (IDLE → READY → ASCENT → DESCENT → LANDED), bez kļūdām rādījumi pieaug un pēc nosēšanās atgriežas uz nulli. | Strādā. Tiek veiksmīgi pārslēgti stāvokļi un netiek uzrādīta neviena kļūda. |
| Kļūda pirms lidojuma / SAFE režīms | Kļūdas aktivizēšana pirms lidojuma | Sistēma pāriet SAFE režīmā, lidojums nesākas, dati neizmainās. | Strādā. tiek uzrādītas SELF_TEST_FAIL un IMU_FAIL kļūdas. |
| Neatļauta stāvokļu pāreja | Nepareizas stāvokļu secības izsaukšana | Sistēma atpazīst nepareizu secību un uzrāda kļūdu. | Strādā. INVALID_STATE_TRANSITION kļūda tiek uzrādīta. |
| IMU datu zudums un atkopšanās | IMU datu zuduma simulācija | Parādās īslaicīgs datu zudums, pēc tam sistēma atkopjas, un lidojums beidzas veiksmīgi. | Strādā. IMU_DATA_LOST un IMU_RECOVERED tiek korekti uzrādīti. |
| Vairākas vienlaicīgas kļūdas | Vairāku kļūdu vienlaicīga aktivizēšana | Sistēma apstrādā vairākas kļūdas vienlaikus un turpina darbību. | Strādā. Nolaišanās laikā uzrāda GPS_LOST, BARO_FAIL un LOW_BATTERY kļūdu. Veiksmīga nosēšanās. |
| Sakaru zudums un GPS svārstības | Nestabilas sakaru un GPS darbības simulācija | Sistēma turpina darbību pēc īslaicīgas nestabilitātes. | Strādā. Lidojuma laikā tiek uzrādītas vairākas kļūdas, notiek atkopšanās un veiksmīga nosēšanās. |
| Stāvokļa pārejas kļūda ar atkopšanos | Kļūdas un atkopšanās scenārijs | Sistēma konstatē kļūdu un pēc tam atkopjas. | Strādā. Uzrāda INVALID_STATE_TRANSITION kļūdu, notiek atkopšanās un veiksmīga nosēšanās. |
| Jaukts anomāliju tests | Kombinēts kļūdu scenārijs | Sistēma apstrādā vairākas kļūdas bez darbības pārtraukuma. | Strādā. Lidojuma laikā parādās daudz kļūdas paziņojumu, notiek atkopšanās un veiksmīga nosēšanās. |


---

# Nefunkcionālie testi

| Testa veids | Ievade, darbība | Sagaidāmais rezultāts | Patiesais rezultāts |
|------------|-----------------|----------------------|---------------------|
| Atvēršana no citiem datoriem | Sistēmas atvēršana no cita datora tīklā | Sistēma ir pieejama tīklā un darbojas citos datoros. | Strādā. Sistēmu var atvērt no citiem datoriem. |
| Atvēršana no mobilajām ierīcēm | Sistēmas atvēršana mobilajā ierīcē | Sistēma darbojas mobilajās ierīcēs ar pielāgotu UI. | Nestrādā. aplikācija nav optimizēta mobilajām ierīcēm. |
| Sākumlapas pieejamība | Viena HTTP pieprasījuma nosūtīšana | Sistēma atbild uz pieprasījumu bez kļūdām. | Tika nosūtīts viens HTTP pieprasījums, kas tika apstrādāts veiksmīgi (100% OK, 0% kļūdu). Atbildes laiks bija 340 ms, kas norāda uz pieņemamu reakcijas ātrumu. Tomēr, ņemot vērā, ka tests ietvēra tikai vienu pieprasījumu, nav iespējams izdarīt secinājumus par sistēmas veiktspēju vai stabilitāti pie lielākas slodzes. Šis tests kalpo kā sākotnējais sistēmas pieejamības pārbaudījums (smoke test). |
| Vienmērīga lietotāju slodze | 300 HTTP pieprasījumu nosūtīšana | Sistēma uztur stabilu veiktspēju pie nemainīgas slodzes. | Tika nosūtīti 300 HTTP pieprasījumi, no kuriem visi tika apstrādāti veiksmīgi (100% OK, 0% kļūdu). Vidējais atbildes laiks bija 29 ms, savukārt 95% pieprasījumu tika apstrādāti ātrāk par 66 ms, kas norāda uz ļoti labu sistēmas veiktspēju. Lai gan maksimālais atbildes laiks sasniedza 557 ms, šādi gadījumi bija reti un neietekmē kopējo sistēmas darbību. Rezultāti liecina, ka sistēma ir stabila un spēj efektīvi apkalpot lietotājus pie dotās slodzes. |
| Pieaugoša slodze (Ramp-up) | Pakāpeniski pieaugoša pieprasījumu plūsma | Sistēma saglabā veiktspēju pie pakāpeniskas slodzes pieauguma. | Tika simulēts pakāpeniski pieaugošs lietotāju skaits, nosūtot 100 HTTP pieprasījumus 58 sekunžu laikā. Visi pieprasījumi tika apstrādāti veiksmīgi (100% OK, bez kļūdām). Vidējais atbildes laiks bija 18 ms, savukārt 95% pieprasījumu tika apstrādāti ātrāk par 26 ms, kas liecina par ļoti labu sistēmas veiktspēju arī pie pieaugošas slodzes. Maksimālais atbildes laiks sasniedza 175 ms, tomēr šādi gadījumi bija reti un neietekmēja kopējo sistēmas darbību. Rezultāti norāda, ka sistēma spēj stabili un efektīvi apkalpot lietotājus pie pakāpeniski pieaugošas slodzes bez būtiskas veiktspējas degradācijas. |
| Ilgstoša slodze (Soak test) | Ilgstoša slodze ar 1500 pieprasījumiem | Sistēma ilgstoši darbojas bez veiktspējas pasliktināšanās. | Tika simulēta ilgstoša, nemainīga slodze 5 minūšu laikā, nosūtot 1500 HTTP pieprasījumus. Visi pieprasījumi tika apstrādāti veiksmīgi (100% OK, bez kļūdām). Vidējais atbildes laiks bija 14 ms, savukārt 95% pieprasījumu tika apstrādāti ātrāk par 20 ms, kas norāda uz ļoti stabilu sistēmas darbību ilgstošas slodzes apstākļos. Maksimālais atbildes laiks sasniedza 198 ms, tomēr šādi gadījumi bija reti un neietekmēja kopējo sistēmas veiktspēju. Testa rezultāti neuzrādīja būtisku veiktspējas degradāciju laika gaitā, kas liecina par sistēmas stabilitāti un spēju uzturēt nemainīgu darbību ilgstošas slodzes apstākļos. |
| Pīķa slodze (Spike test) | Straujš pieprasījumu pieaugums (200 vienlaikus) | Sistēma saglabā darbību pie strauja slodzes pieauguma. | Pīķa slodzes testā tika simulēts straujš lietotāju skaita pieaugums, vienlaikus nosūtot 200 pieprasījumus. Visi pieprasījumi tika apstrādāti veiksmīgi (100% OK, bez kļūdām), kas liecina par sistēmas stabilitāti pat pie augstas slodzes. Tomēr vidējais atbildes laiks būtiski palielinājās līdz 1115 ms, un lielākā daļa pieprasījumu tika apstrādāti 800–1200 ms intervālā. Tas norāda, ka sistēma nespēj uzturēt augstu veiktspēju pie pēkšņas slodzes pieauguma, lai gan tā saglabā funkcionalitāti. Rezultāti liecina, ka sistēma ir stabila, bet ne optimizēta darbam pīķa slodzes apstākļos. |
| Statisko resursu tests | Statisko resursu pieprasījumu nosūtīšana | Visi resursi (HTML, CSS, attēli) ielādējas korekti. | Statisko resursu testā tika pārbaudīta sākumlapas, CSS un favicon ielāde, kopumā nosūtot 150 HTTP pieprasījumus. Visi pieprasījumi tika apstrādāti veiksmīgi (100% OK, bez kļūdām). Vidējais atbildes laiks bija 30 ms, un 95% pieprasījumu tika apstrādāti ātrāk par 34 ms, kas liecina par labu sistēmas veiktspēju statisko resursu piegādē. Tomēr tika novēroti atsevišķi gadījumi ar augstāku atbildes laiku (līdz 1293 ms), kas var būt saistīti ar kešošanas neizmantošanu vai pirmreizēju resursu ielādi. Kopumā sistēma spēj efektīvi piegādāt statiskos resursus, nodrošinot ātru lietotāja pieredzi. |
   
   
## Šeit pieejams kods izstrādātajām Gatling simulācijām, kas tika izmantotas, lai testētu sistēmas slodzes panesamību: [VERTEX simulācijas](TESTESANA-AR-GATLING)
   
  
---

# Integrācijas testi

## 1. tests: Pārbaudīt, vai CSV ielādes modulis korekti nodod datus parserim un parseris korekti nodod tos GUI attēlošanai.
Ievade / darbība: Tiek ielādēts CSV fails ar šādu saturu:
time,ax,ay,az,altitude
  0.0,0.01,0.02,9.81,0.0  
  0.1,0.03,0.01,9.84,0.5  
  0.2,0.10,0.05,10.20,2.0  
  0.3,0.20,0.07,11.30,5.5  
  0.4,0.15,0.04,10.80,8.0  
  
**Sagaidāmais rezultāts:** CSV fails tiek nolasīts, parseris korekti sadala laukus, un GUI grafikos tiek attēlotas atbilstošās ax, ay, az un altitude vērtības bez kļūdām.  
**Rezultāts:** dati tiek korekti attēloti GUI.
  
---
## 2. tests: Pārbaudīt, vai telemetrijas datu plūsma no “fake source” tiek korekti apstrādāta un reāllaikā atjauno GUI grafikus.
Ievade / darbība: Tiek izmantots datu avots, kas periodiski sūta šādas rindas:
  0.0,0.12,-0.05,9.79,0.0  
  0.1,0.08,0.02,9.85,0.3  
  0.2,0.15,0.01,10.10,1.1  
  0.3,0.30,0.10,11.00,3.8  
  0.4,0.25,0.04,10.60,6.4  
  
**Sagaidāmais rezultāts:** GUI saņem jaunos datus, grafiki mainās katrā jaunajā solī un programma neuzkaras.  
**Rezultāts:** Grafiki tiek atjaunināti, un GUI saglabā darbspēju.
  
---
## 3. tests: Pārbaudīt, vai parseris un GUI korekti apstrādā nepareizā secībā saņemtus telemetrijas ierakstus.
Ievade / darbība: Tiek padoti šādi dati sajauktā secībā:
time,ax,ay,az,altitude  
 0.3,0.20,0.07,11.30,5.5  
 0.1,0.03,0.01,9.84,0.5  
 0.4,0.15,0.04,10.80,8.0  
 0.0,0.01,0.02,9.81,0.0  
 0.2,0.10,0.05,10.20,2.0  
  
**Sagaidāmais rezultāts:** Sistēma vai nu sakārto ierakstus pēc time lauka, vai ignorē nekorekti pienākušos punktus, bet GUI neuzkaras un neattēlo acīmredzami bojātu grafiku.  
**Rezultāts:** GUI un parsera savienojums saglabā darbspēju arī pie sajauktas secības datiem.
  
---
## 4. tests: Pārbaudīt, vai parseris un GUI korekti apstrādā rindas ar trūkstošiem datu laukiem.
Ievade / darbība: Tiek ielādēts CSV fails ar trūkstošu ay vērtību vienā ierakstā:
time,ax,ay,az,altitude  
 0.0,0.01,0.02,9.81,0.0  
 0.1,0.03,0.01,9.84,0.5  
 0.2,0.10,,10.20,2.0  
 0.3,0.20,0.07,11.30,5.5  
 0.4,0.15,0.04,10.80,8.0  
  
**Sagaidāmais rezultāts:** Parseris neatstāj sistēmu nekorektā stāvoklī, GUI necrasho, un trūkstošā vērtība tiek vai nu atzīmēta kā null, vai arī konkrētais punkts netiek attēlots.  
**Rezultāts:** Nepilnīgie dati neizraisa programmas avāriju, un GUI turpina darboties.
  
---
## 5. tests: Pārbaudīt, vai lielas datu plūsmas gadījumā parseris, datu glabāšana atmiņā un GUI grafiku atjaunošana darbojas kopā bez kritiskas aiztures.
Ievade / darbība: Tiek padota augstas intensitātes datu plūsma, piemēram:
time,ax,ay,az,altitude  
 0.00,0.10,0.02,9.80,0.0  
 0.01,0.11,0.03,9.82,0.1  
 0.02,0.12,0.02,9.85,0.3  
 0.03,0.18,0.04,10.10,0.7  
 0.04,0.22,0.06,10.80,1.4  
 ...  
 9.99,-0.05,0.01,9.70,0.0  
  
**Sagaidāmais rezultāts:** Datu plūsma tiek apstrādāta, GUI paliek lietojams, grafiki tiek atjaunoti un programma nekrīt pat pie ievērojami lielāka datu apjoma.  
**Rezultāts:** Sistēma apstrādā lielāku datu apjomu bez kritiskas kļūmes.
  
---
## 6. tests: Pārbaudīt, vai sensoru datu apvienošanas modulis korekti sadarbojas ar GUI stāvokļa attēlojumu.
Ievade / darbība: Tiek padoti sintētiski IMU un GNSS dati lineārai kustībai:
time,ax,ay,az,gnss_x,gnss_y,gnss_z  
 0.0,0.00,0.00,9.81,0.0,0.0,0.0  
 0.1,0.20,0.00,9.81,0.5,0.0,0.0  
 0.2,0.20,0.00,9.81,1.0,0.0,0.0  
 0.3,0.20,0.00,9.81,1.5,0.0,0.0  
 0.4,0.20,0.00,9.81,2.0,0.0,0.0  
  
**Sagaidāmais rezultāts:** Sensoru apvienošanas modulis aprēķina stabilu stāvokļa novērtējumu, un GUI šo novērtējumu attēlo bez lēcieniem, kļūdām vai datu sabrukuma.  
**Rezultāts:** Datu apvienošanas rezultāts tiek korekti nodots GUI attēlošanai.

  
---
# Secinājumi un ieteikumi

## Secinājumi

Programmas testēšanas rezultāti parāda, ka VERTEX sistēma kopumā darbojas korekti un izpilda tai paredzētās pamatfunkcijas. Funkcionālie testi apliecina, ka sistēma spēj ielādēt CSV failus, korekti attēlot lidojuma stāvokļus un apstrādāt dažādus kļūdu scenārijus, nodrošinot arī veiksmīgu atkopšanos.

Nefunkcionālo testu rezultāti liecina, ka sistēma ir stabila pie normālas, pakāpeniski pieaugošas un ilgstošas slodzes. Tiek uzturēts zems atbildes laiks un netiek novērotas kļūmes. Tomēr pīķa slodzes apstākļos novērojama būtiska veiktspējas pasliktināšanās, kas norāda, ka sistēma saglabā funkcionalitāti, bet nav pilnībā optimizēta straujam lietotāju pieaugumam.

Integrācijas testi apstiprina, ka galvenie sistēmas moduļi – CSV ielāde, datu parsēšana un GUI attēlošana – savstarpēji darbojas korekti arī nestandarta situācijās. Sistēma saglabā stabilitāti pie nepilnīgiem datiem, sajauktas secības un lielākas datu plūsmas.

Kopumā sistēma ir funkcionāli pilnvērtīga un stabila, ar labu pamatu turpmākai attīstībai.

## Ieteikumi

- Ieteicams ieviest iespēju atkārtoti palaist testu ar to pašu CSV failu bez lapas pārlādes, piemēram, pievienojot Refresh vai Reset funkciju. Tas būtiski uzlabotu lietojamību.

- Nepieciešams optimizēt sistēmas darbību pīķa slodzes apstākļos, lai samazinātu atbildes laiku pie strauja lietotāju skaita pieauguma.

- Būtu vēlams uzlabot statisko resursu ielādi, ieviešot "cache" mehānismus un optimizējot pirmreizējo ielādi.

- Ieteicams apsvērt responsīva dizaina ieviešanu, ja nākotnē plānota sistēmas izmantošana mobilajās ierīcēs.
