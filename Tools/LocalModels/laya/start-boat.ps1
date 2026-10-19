param([string]$ProjectPath='')
$ErrorActionPreference='Stop'
if (!$ProjectPath) { $ProjectPath = (Get-Content "$PSScriptRoot\project.json" -Raw | ConvertFrom-Json).project }
if (!(Test-Path -LiteralPath (Join-Path $ProjectPath 'jev_api\boat_client.py'))) { throw 'Pass -ProjectPath pointing to the cloned Unity project.' }
. "$PSScriptRoot\environment.ps1"
$health=Invoke-RestMethod http://127.0.0.1:8010/deployment -TimeoutSec 10
$manifest=Get-Content "$PSScriptRoot\models.json" -Raw | ConvertFrom-Json
if ($health.revision -ne $manifest.revision -or $health.device -notlike 'cuda*') { throw 'Start the pinned Laya GPU service first.' }
$logFile="$PSScriptRoot\logs\boat-$(Get-Date -Format 'yyyyMMdd-HHmmss').jsonl"
& "$PSScriptRoot\.venv\Scripts\python.exe" "$ProjectPath\jev_api\boat_client.py" --backend laya --log $logFile
exit $LASTEXITCODE
