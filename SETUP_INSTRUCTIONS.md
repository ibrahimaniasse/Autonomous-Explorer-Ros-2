# Setup Instructions — Antigravity Workspace

## Étape 1 : Créer le workspace

```bash
mkdir autonomous-explorer-ros2
cd autonomous-explorer-ros2
git init
```

## Étape 2 : Copier les fichiers de configuration

Place les fichiers suivants à la racine du projet :

```
autonomous-explorer-ros2/
├── GEMINI.md                          ← Fichier principal (rules)
├── .agent/
│   └── workflows/
│       ├── phase1.md                  ← /phase1 slash command
│       ├── phase2.md                  ← /phase2 slash command
│       ├── phase3.md                  ← /phase3 slash command
│       └── phase4.md                  ← /phase4 slash command
```

## Étape 3 : Ouvrir dans Antigravity

1. Ouvre Antigravity
2. File → Open Folder → sélectionne `autonomous-explorer-ros2/`
3. Vérifie que les rules sont chargées :
   - Clique sur `...` en haut à droite du chat agent → Customizations → Rules
   - Tu devrais voir le contenu de GEMINI.md listé

## Étape 4 : Lancer Phase 1

Dans la **Manager View** (Agent Manager), tape :

```
/phase1
```

L'agent va planifier, puis exécuter. Vérifie les artifacts à chaque étape.

## Étape 5 : Itérer

Une fois Phase 1 terminée et testée :
- `/phase2` → Perception & SLAM
- `/phase3` → Exploration autonome
- `/phase4` → Documentation & CI

## Tips

- **Modèle recommandé** : Gemini 3.1 Pro pour les phases 1-3, Flash pour la phase 4 (documentation).
- **Si rate-limité** : Bascule sur Claude Sonnet 4.6 (supporté nativement dans Antigravity).
- **Test local** : L'agent ne peut pas lancer Gazebo dans son browser. Tu dois tester avec `docker compose up` sur ta machine.
- **Enregistrer le GIF** : Une fois Phase 3 fonctionnelle, lance la simulation, utilise `peek` ou OBS pour capturer un GIF de 15-20s du robot en exploration, et remplace le placeholder dans le README.
