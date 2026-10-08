# Lance la surveillance webcam (Logitech C270) sur le PC.
# Docker n'a pas acces a l'USB : ce service doit tourner ICI, pas dans un conteneur.

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    Write-Host "Creation de l'environnement Python..."
    py -3 -m venv .venv
    if ($LASTEXITCODE -ne 0) { python -m venv .venv }
}

$py = ".\.venv\Scripts\python.exe"
& $py -m pip install --upgrade pip
& $py -m pip install -r requirements.txt

Write-Host ""
# CAMERA_NAME=C270   -> choisit la Logitech (pas la webcam HP du PC)
# CAMERA_INDEX=1     -> force un numero si besoin
Write-Host "Webcam : http://localhost:8001  (flux /video, etat /status)"
Write-Host "Photos autorisees : .\known_faces\<prenom>\*.jpg"
Write-Host ""
& $py app.py
