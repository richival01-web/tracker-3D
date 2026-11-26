# CLAUDE.md – 3D Multi-Object Tracker in ROS 2

Istruzioni per Claude Code in questo repository. Rispondi in italiano; codice, commenti, messaggi di commit e README in inglese.

## Contesto

Progetto personale di Riccardo Valentini (M.Sc. Automotive Engineering, TUM), pensato per il portfolio e per le candidature a tesi o working student. È un tracker 3D *tracking-by-detection*:

- filtro di Kalman, associazione con algoritmo ungherese, ciclo di vita delle tracce
- tracker come nodo ROS 2 in C++17, valutato con le metriche ufficiali nuScenes (AMOTA, AMOTP, ID switch)
- in più, uno studio di robustezza con detection degradate

Riccardo lavora 1,5–2 ore al giorno, di solito la sera. Obiettivi:

- **MVP pubblico entro domenica 6.12.2026** (tag v0.1)
- **v1.0 entro il 20.12.2026**

Pipeline:
1. nuScenes v1.0-mini letto con `nuscenes-devkit`
2. Simulatore di detection (Python): box ground truth + rumore configurabile
3. Tracker (C++): predict → associazione → update → nascita e morte delle tracce
4. Output: tracce con ID in RViz2/Foxglove + JSON in formato nuScenes tracking
5. Valutazione con il devkit ufficiale su `mini_val`

Fuori scope: addestrare un detector, camere, dataset completo.

## Come lavorare con Riccardo (importante)

- **Deve saper spiegare ogni riga del repo** in un colloquio. Quindi:
  - spiega il perché delle scelte
  - preferisci passi piccoli che lui possa rileggere, a grossi blocchi generati in una volta
  - dopo ogni pezzo non banale, riassumi in 2–3 frasi cosa fa e quali errori evita
- **Kalman, algoritmo ungherese e gestione delle tracce sono il cuore del progetto.** Guidalo:
  - spiega l'algoritmo
  - proponi le firme e i test
  - fai scrivere a lui l'implementazione e poi rivedila
  - scrivi codice completo solo se te lo chiede esplicitamente
- Per Docker, CMake, boilerplate ROS, script di I/O e di plotting puoi essere più diretto.
- All'inizio di una sessione:
  - guarda la sezione "Stato" in fondo e la data di oggi
  - proponi il compito del giorno, o il recupero dei giorni rimasti indietro
- A fine compito:
  - verifica il criterio "fatto quando"
  - aggiorna la sezione "Stato"
  - proponi un messaggio di commit
- Non fare push e non creare tag senza chiedere.

## Regole del progetto (non negoziabili)

- **Niente codice, parametri o dati** dei corsi TUM (Autonomous Systems, F1TENTH) né della cattedra FTM/EDGAR.
- **Niente dati nuScenes nel repo**, nemmeno piccoli estratti, JSON di esempio con token reali o immagini:
  - il percorso del dataset arriva dalla variabile d'ambiente `NUSC_ROOT` (montata come volume in Docker)
  - prima di ogni commit controlla `git status`
- nuScenes è solo per uso non commerciale: cita sempre il paper (Caesar et al., CVPR 2020).
- AB3DMOT (arXiv 1907.03961) e SimpleTrack (arXiv 2111.09621) sono **riferimenti da leggere, non da copiare**. Si possono citare le idee, ma non incollare il loro codice.
- Licenza MIT per il codice. Dipendenze con licenza compatibile.

## Stack

- ROS 2 Jazzy in Docker (Ubuntu 24.04), colcon
- C++17, Eigen 3, GoogleTest; build con `ament_cmake`
- Python 3 con `nuscenes-devkit`, `numpy`, `pyquaternion`
- `vision_msgs/Detection3DArray` per detection e tracce; `visualization_msgs/MarkerArray` per la visualizzazione
- RViz2 (nel container, via X11; scelto al posto di Foxglove il 26.11); GitHub Actions dalla fase 2

## Struttura prevista del repo

```
docker/Dockerfile
src/tracker_core/     # C++ library, NO ROS dependency in the core (only ament_cmake to build)
  include/tracker_core/  src/  test/
src/tracker_ros/      # (from G9) C++ ROS 2 node + launch files, depends on tracker_core
src/nusc_tools/       # Python: loader, detection simulator node, JSON export, evaluation
scripts/              # helper scripts (eval, plots, GIF)
docs/                 # pipeline diagram, GIF, plots
```

## Convenzioni tecniche (errori tipici da evitare)

- **Frame:**
  - il tracker lavora nel frame **globale** di nuScenes
  - i box del devkit da `get_boxes` / `get_sample_data` possono essere nel frame del sensore o dell'ego: verifica sempre in che frame sei
- **Unità:**
  - metri, radianti, secondi
  - i timestamp nuScenes sono in **microsecondi**: converti a secondi per calcolare `dt`
  - i keyframe sono a ~2 Hz (`dt` ≈ 0,5 s, ma usa sempre il `dt` reale)
- **Dimensioni:** il devkit usa l'ordine **`wlh`** (width, length, height). Il formato di submission usa `size = [w, l, h]`. Non confondere con `lwh`.
- **Rotazioni:**
  - i quaternioni nuScenes/pyquaternion sono `[w, x, y, z]`, mentre `geometry_msgs/Quaternion` è `x, y, z, w`
  - yaw = rotazione attorno a z
  - nel Kalman normalizza l'innovazione dell'angolo in [-π, π]
  - valuta di correggere i flip di ~π, che nelle detection rumorose sono frequenti
- **Stato del filtro** (stile AB3DMOT): `[x, y, z, yaw, l, w, h, vx, vy, vz]`, modello a velocità costante. Q e R sono parametri, non numeri inseriti nel codice.
- **Classi tracking nuScenes:** `bicycle, bus, car, motorcycle, pedestrian, trailer, truck`. Associa solo detection e tracce della stessa classe.
- **Riproducibilità:** il simulatore usa un seed fisso (parametro ROS); due run con lo stesso seed devono essere identiche.
- **Formato dei risultati** (`results.json`): `meta` (`use_camera`, `use_lidar`, `use_radar`, `use_map`, `use_external`) + `results` con chiave `sample_token`. Per ogni traccia:
  - `sample_token`, `translation`, `size`, `rotation`
  - `velocity` (2D)
  - `tracking_id`, `tracking_name`, `tracking_score`
- **Split:** `mini_train` (8 scene) e `mini_val` (2 scene). Valuta su `mini_val`.

## Comandi utili

```bash
# build the image (from the repo root; pass host UID/GID so files in the volume are yours)
docker build -f docker/Dockerfile -t tracker3d:dev \
  --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) docker/
# allow X11 windows from your own user (once per login session, on the host)
xhost +SI:localuser:$(id -un)
# start the container: repo mounted as colcon workspace, dataset read-only,
# X11 socket + /dev/dri (AMD iGPU via Mesa) for RViz2
docker run --rm -it --name tracker3d --hostname tracker3d \
  -v "$PWD":/ws -v "$NUSC_ROOT":/data/nuscenes:ro \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --device /dev/dri \
  --group-add "$(getent group video | cut -d: -f3)" \
  --group-add "$(getent group render | cut -d: -f3)" \
  tracker3d:dev
# if RViz2 shows a black window / OpenGL errors: add -e LIBGL_ALWAYS_SOFTWARE=1
# build and test (inside the container)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && colcon test && colcon test-result --verbose
# official tracking evaluation (inside the container, NUSC_ROOT=/data/nuscenes)
python3 -m nuscenes.eval.tracking.evaluate --result_path results/results.json \
  --output_dir results/eval --eval_set mini_val --version v1.0-mini --dataroot $NUSC_ROOT
```
Note:
- `ros2 pkg create` genera lo scheletro dei pacchetti, ma va riletto: per esempio scrive `find_package(eigen)` invece di `Eigen3` e non imposta C++17.
- I nodi Python di ROS girano con `/usr/bin/python3`: le dipendenze pip vanno nel Python di sistema del container, non in un venv.
- Per i pacchetti `ament_python` serve `extras_require={'test': ['pytest']}`, altrimenti colcon usa `unittest` e fallisce se non trova test.

(Aggiorna questa sezione quando esistono i launch file, da G9.)

## Programma giornaliero – Fase 1: MVP

| Giorno | Data | Compito | Fatto quando |
| --- | --- | --- | --- |
| G1 | Lun 23.11 | Repo GitHub pubblico (MIT, README provvisorio, .gitignore); registrazione nuScenes e download v1.0-mini | Repo online, download partito |
| G2 | Mar 24.11 | Dockerfile con ROS 2 Jazzy + Eigen + GoogleTest; workspace colcon con pacchetti vuoti `tracker_core` (C++) e `nusc_tools` (Python) | `colcon build` passa nel container |
| G3 | Mer 25.11 | Tutorial nuscenes-devkit; script che per una scena stampa le box annotate di ogni keyframe in coordinate globali con timestamp | Box di una scena stampate con timestamp |
| G4 | Gio 26.11 | Nodo Python che pubblica le box GT come `Detection3DArray` + `MarkerArray` al ritmo dei keyframe | Box visibili in RViz2/Foxglove |
| G5 | Ven 27.11 | Rumore configurabile via parametri ROS (σ posizione e yaw, prob. oggetto mancato, falsi positivi, score), seed fisso | Due run con lo stesso seed identiche |
| G6 | Sab 28.11 | Filtro di Kalman a velocità costante in C++/Eigen (stato stile AB3DMOT), predict/update; test GoogleTest su moto rettilineo | Test verde |
| G7 | Dom 29.11 | Costo di associazione (distanza tra centri) + algoritmo ungherese scritto da Riccardo, test su casi piccoli noti | Test verdi |
| G8 | Lun 30.11 | Gestione tracce: nascita, conferma (min hits), morte (max age), associazione per classe; tutto nella libreria senza ROS | Test su sequenza sintetica con un oggetto che entra ed esce |
| G9 | Mar 01.12 | Nodo ROS 2 C++: sottoscrive detection, pubblica tracce e marker con ID; launch file unico simulatore + tracker | Pipeline avviata con un solo `ros2 launch` |
| G10 | Mer 02.12 | Run end-to-end su una scena; debug di frame, timestamp e yaw | ID stabili e tracce coerenti a vista |
| G11 | Gio 03.12 | Export dei risultati in JSON formato nuScenes tracking; GIF della demo | JSON valido + GIF di 10–20 s |
| G12 | Ven 04.12 | Valutazione ufficiale con il devkit su `mini_val` (AMOTA, AMOTP, ID switch), senza rumore e con rumore base | Tabella con i primi numeri |
| G13 | Sab 05.12 | README: descrizione, schema pipeline, GIF, tabella risultati, avvio con un comando in Docker, crediti e licenza nuScenes | README completo |
| G14 | Dom 06.12 | Pulizia, controllo delle regole, tag v0.1 | Repo pubblicabile |

## Programma – Fase 2: rifinitura (~1 h al giorno)

| Giorno | Data | Compito |
| --- | --- | --- |
| G15 | Lun 07.12 | (fuori dal repo) solleciti per le candidature con il link al repo; CV aggiornato |
| G16–G17 | 08–09.12 | Costo di associazione 3D IoU/GIoU; confronto con la distanza tra centri |
| G18–G19 | 10–11.12 | Regolazione dei parametri (soglie, min hits, max age, rumore di processo) |
| G20–G22 | 12–14.12 | Sweep di 4–5 livelli di rumore; grafico AMOTA vs livello |
| G23–G24 | 15–16.12 | GitHub Actions: build + test nel container |
| G25–G26 | 17–18.12 | README finale con grafico di robustezza; tag v1.0 |
| G27–G28 | 19–20.12 | Margine per recuperi |

## Stato

Aggiorna questa sezione a fine compito (giorno, data, cosa è fatto, cosa resta aperto).

- 23.11.2026: G1 completato e chiuso. Repo pubblico su GitHub (MIT, README provvisorio, .gitignore), commit + push del .gitignore fatti (`main` allineato con `origin/main`). CLAUDE.md sta in `Personal_projects/`, fuori dal repo. nuScenes v1.0-mini estratto in `Personal_projects/datasets/nuscenes/` (fuori dal repo; `maps/`, `samples/`, `sweeps/`, `v1.0-mini/` direttamente lì dentro). `NUSC_ROOT` impostata in `~/.zshrc` su quella cartella. Verifica con nuscenes-devkit 1.2.0 (venv temporaneo, poi rimosso): 10 scene, 404 sample, mini_train 8/8, mini_val 2/2, nessun file di keyframe mancante. Nota: Ubuntu 24.04 blocca `pip install` di sistema (PEP 668) → devkit nel Docker.
- 23.11.2026: storia git riscritta con l'email no-reply di GitHub (`239980864+richival01-web@users.noreply.github.com`, anche in `git config user.email` del repo) e force push fatto.
- 23.11.2026: G2 completato (in anticipo di un giorno). `docker/Dockerfile`:
  - base `ros:jazzy-ros-base`
  - Eigen, GoogleTest, `ament_cmake_gtest`, `vision_msgs`, `visualization_msgs`, pytest
  - `nuscenes-devkit` 1.2.0 + pyquaternion con pip nel Python di sistema, `numpy<2`
  - utente non-root `dev` con UID/GID dell'host

  Pacchetti vuoti:
  - `tracker_core` (`ament_cmake`, C++17, Eigen3, nessuna dipendenza ROS; cartelle `include/`, `src/`, `test/` con `.gitkeep`)
  - `nusc_tools` (`ament_python`, smoke test `test_import.py`)

  Nel container `colcon build` passa per i 2 pacchetti; `colcon test`: 2 test, 0 fallimenti. Commit `147508d` pushato (`main` allineato con `origin/main`). Nessuna voce aperta. Prossimo: G3 (tutorial devkit + box globali con timestamp), previsto per mer 25.11, ma si può anticipare a domani.
- 24.11.2026: G3 completato (in anticipo di un giorno).
  - `nusc_tools/scene_loader.py`: `load_scene_keyframes(nusc, scene)` segue la lista dei sample, converte i timestamp da µs a s, legge le `sample_annotation` (già nel frame globale), tiene solo le 7 classi di tracking (`category_to_tracking_name` del devkit) e scarta le box con 0 punti lidar+radar come la valutazione ufficiale. Dataclass `Keyframe` / `GtBox` (size `wlh`, quaternione `wxyz`, yaw, velocità 2D, `instance_token`). Da riusare nel G4.
  - `nusc_tools/print_scene.py` (console script `print_scene`): `ros2 run nusc_tools print_scene --scene scene-0103 --max-frames 3`.
  - `test/test_scene_loader.py`: 4 test con un finto dataset (nessun dato nuScenes). `colcon test`: 6 test, 0 fallimenti.
  - Dockerfile: aggiunti `motmetrics==1.4.0` e `pandas<2.3`, necessari a `nuscenes.eval.tracking`. Il devkit 1.2.0 richiede la 1.4.0 su Python ≥ 3.10; la 1.1.3 si rompe su 3.12. Immagine ricostruita e smoke test dell'accumulatore MOT del devkit ok.
  - Osservazioni: mini_val = scene-0103 (40 keyframe, 19,4 s) e scene-0916 (41 keyframe, 19,9 s), `dt` ≈ 0,500 s. Alcuni oggetti spariscono per un keyframe (0 punti) e poi ricompaiono, il che è rilevante per `max_age` nel G8.
  - Da fare: commit (non ancora fatto). Prossimo: G4 (publisher GT come `Detection3DArray` + `MarkerArray`).
