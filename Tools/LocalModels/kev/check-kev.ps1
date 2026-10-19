$ErrorActionPreference='Stop'
$manifest=Get-Content "$PSScriptRoot\models.json" -Raw | ConvertFrom-Json
$health=Invoke-RestMethod http://127.0.0.1:8009/v1/models -TimeoutSec 10
$card=$health.models[0]
if ($card.run -ne $manifest.kev_path -or $card.base -ne $manifest.base_id -or $card.device -notlike 'cuda*' -or $card.truncate_states) { throw 'Unexpected deployed model/device/truncation configuration.' }
$card | Select-Object name,run,base,device,backend,dtype,truncate_states | ConvertTo-Json
