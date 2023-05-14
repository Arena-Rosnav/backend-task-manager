# Getting Started

This taskmanager is used to run processes from the backend.

Ask a team member for the current version of the `.env` file.

Put this file in the root folder of this repository (backend_task_manager) and check that the `.env` file contains the necessary information for the database, endpoints and keys.

Set up Poetry and install the workspace. Then start the Poetry Shell.

Run task_manager inside the Poetry Shell with:

```bash
python3 task_manager.py
```

See if tasks are running in docker with:

```bash
docker ps -a
```

Detailed overview of the architecture
[Figma](https://www.figma.com/file/LUsAHpY9fHezP4Fhz87cym/Arena-Web?type=design&node-id=0-1)
