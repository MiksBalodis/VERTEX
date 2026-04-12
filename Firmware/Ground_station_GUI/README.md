# Šeit ir viss nepieciešamais zemes stacijas GUI testēšanai

---

# Sagatavošanās

1) Lejupieladēt repozitoriju
2) Instalēt [Docker](https://www.docker.com)
3) Ja nav ieslēgta virutalizācija, ieslēgt
4) Atvērt *Docker Desktop*
5) Palaist *run_docker.bat*
6) Pārlūkprogrammā atvērt [localhost:5000](http://localhost:5000)
   
## Get started

Install the dependencies...

```bash
cd svelte-app
npm install
```

...then start [Rollup](https://rollupjs.org):

```bash
npm run dev
```

Navigate to [localhost:8080](http://localhost:8080). You should see your app running. Edit a component file in `src`, save it, and reload the page to see your changes.

By default, the server will only respond to requests from localhost. To allow connections from other computers, edit the `sirv` commands in package.json to include the option `--host 0.0.0.0`.

If you're using [Visual Studio Code](https://code.visualstudio.com/) we recommend installing the official extension [Svelte for VS Code](https://marketplace.visualstudio.com/items?itemName=svelte.svelte-vscode). If you are using other editors you may need to install a plugin in order to get syntax highlighting and intellisense.

## Building and running in production mode

To create an optimised version of the app:

```bash
npm run build
```

You can run the newly built app with `npm run start`. This uses [sirv](https://github.com/lukeed/sirv), which is included in your package.json's `dependencies` so that the app will work when you deploy to platforms like [Heroku](https://heroku.com).


## Single-page app mode

By default, sirv will only respond to requests that match files in `public`. This is to maximise compatibility with static fileservers, allowing you to deploy your app anywhere.

If you're building a single-page app (SPA) with multiple routes, sirv needs to be able to respond to requests for *any* path. You can make it so by editing the `"start"` command in package.json:

```js
"start": "sirv public --single"
```

## Using TypeScript

This template comes with a script to set up a TypeScript development environment, you can run it immediately after cloning the template with:

```bash
node scripts/setupTypeScript.js
```

Or remove the script via:

```bash
rm scripts/setupTypeScript.js
```

If you want to use `baseUrl` or `path` aliases within your `tsconfig`, you need to set up `@rollup/plugin-alias` to tell Rollup to resolve the aliases. For more info, see [this StackOverflow question](https://stackoverflow.com/questions/63427935/setup-tsconfig-path-in-svelte).

## Deploying to the web

### With [Vercel](https://vercel.com)

Install `vercel` if you haven't already:

```bash
npm install -g vercel
```

Then, from within your project folder:

```bash
cd public
vercel deploy --name my-project
```

### With [surge](https://surge.sh/)

Install `surge` if you haven't already:

```bash
npm install -g surge
```

Then, from within your project folder:

```bash
npm run build
surge public my-project.surge.sh
```


## GUI testēšana ar pievienotajiem `.csv` failiem

Šie testa faili ir paredzēti zemes stacijas GUI pārbaudei ar iepriekš sagatavotiem lidojuma datiem.  
Visiem failiem ir vienāda kolonnu struktūra:

- `time_ms`
- `state`
- `pitch`
- `yaw`
- `roll`
- `altitude`
- `speed`
- `acceleration`
- `error_flags`
- `status_text`

## Kā veikt testēšanu

1. Palaid GUI, kā aprakstīts iepriekš.
2. Ielādē vienu no testa `.csv` failiem.
3. Pārbaudi, vai:
   - visi lauki tiek attēloti;
   - indikatori mainās laikā atbilstoši datiem;
   - stāvokļi (`state`) mainās loģiskā secībā;
   - kļūdu indikatori parādās tad, kad `error_flags`;
   - `status_text` atbilst attiecīgajam brīdim datu plūsmā;
   - pēc faila beigām GUI  rāda pēdējo stāvokli korekti.
4. Atkārto to pašu ar visiem testa failiem.

---

## Sagaidāmā vispārīgā uzvedība

Ja GUI darbojas pareizi, tad:

- laiks (`time_ms`) progresē uz priekšu;
- `state` vērtības tiek attēlotas skaidri un bez nezināmu stāvokļu kļūdām;
- `error_flags = NONE` nozīmē normālu režīmu bez aktīvām kļūdām;
- kombinētās probēlmas, piemēram `GPS_LOST|BARO_FAIL|LOW_BATTERY`, tiek attēloti kā vairākas vienlaicīgas kļūmes;
- pēdējā rindiņā GUI jāatspoguļo faila pēdējais stāvoklis;
- ja lidojums beidzas ar `LANDED`, gala stāvoklim jābūt stabilam, ar nulles vai gandrīz nulles kustības rādītājiem;
- kļūdu scenārijos GUI nedrīkst “salūzt” tikai tāpēc, ka parādās kļūdu karogi vai neparasta stāvokļu secība.

---

## Testa faili un sagaidāmie rezultāti

## Īss sagaidāmais rezultāts katram testa failam

### `test1.csv`
Normāls pilns lidojums.  
Sagaidāms: `IDLE → READY → ASCENT → DESCENT → LANDED`, bez kļūdām, rādījumi pieaug un pēc nosēšanās atgriežas uz nulli.

### `test2.csv`
Kļūme pirms lidojuma / drošais režīms.  
Sagaidāms: sistēma pāriet no `IDLE` uz `SAFE`, lidojums nesākas, visi kustības rādītāji paliek uz nulli, tiek parādīta kļūme.

### `test3.csv`
Neatļauta stāvokļu pāreja.  
Sagaidāms: dati ielādējas, bet GUI parāda kļūdu par nepareizu stāvokļu pāreju, jo lidojums sākas bez korektas iepriekšējās secības.

### `test4.csv`
IMU datu zudums un atkopšanās.  
Sagaidāms: normāls lidojums, uz brīdi parādās IMU datu zudums, pēc tam atkopšanās, un lidojums beidzas veiksmīgi.

### `test5.csv`
Vairākas vienlaicīgas kļūdas.  
Sagaidāms: normāls lidojums, bet GUI vienlaikus parāda vairākus kļūdu karogus, pēc tam lidojums noslēdzas ar nosēšanos.

### `test6.csv`
Sakaru zudums un GPS svārstības.  
Sagaidāms: lidojums turpinās, parādās īslaicīgs sakaru zudums un GPS nestabilitāte, pēc tam atkopšanās un veiksmīga nosēšanās.

### `test7.csv`
Stāvokļa pārejas kļūda ar vēlāku atkopšanos.  
Sagaidāms: GUI parāda neatļautu stāvokļa pāreju, pēc tam atkopšanos, un lidojums tomēr noslēdzas korekti.

### `test8.csv`
Jaukts anomāliju tests.  
Sagaidāms: GUI apstrādā vairākas dažādas kļūdas un brīdinājumus vienā lidojumā, neuzkaras un beigās korekti parāda `LANDED`.

---

## Ko uzskatīt par kļūdu GUI pusē

Testēšanas laikā par problēmu jāuzskata, ja notiek kaut kas no zemāk minētā:

- `.csv` fails netiek ielādēts, lai gan formāts ir korekts;
- `error_flags` netiek attēloti vai tiek attēloti nepareizi;
- kombinētās kļūdas netiek sadalītas vai saprotami parādītas;
- `state` netiek mainīts atbilstoši datiem;
- GUI uzkaras vai avarē kļūdu scenārijos;
- gala stāvoklis neatbilst pēdējai faila rindiņai;
- `SAFE` vai `LANDED` scenārijos GUI joprojām rāda it kā notiktu aktīvs lidojums.

---
