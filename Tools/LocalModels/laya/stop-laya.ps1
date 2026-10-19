$ErrorActionPreference='Stop'
$stateFile="$PSScriptRoot\service-process.json"
if (!(Test-Path $stateFile)) { Write-Output 'No managed Laya service recorded.'; exit 0 }
$state=Get-Content $stateFile -Raw | ConvertFrom-Json
$p=Get-Process -Id $state.pid -ErrorAction SilentlyContinue
if ($null -eq $p) { Remove-Item -LiteralPath $stateFile; Write-Output 'Laya already stopped.'; exit 0 }
if ($p.StartTime.ToUniversalTime().Ticks.ToString() -ne $state.start_ticks) { throw 'PID reused; refusing to stop an unrelated process.' }
$details=Get-CimInstance Win32_Process -Filter "ProcessId=$($state.pid)"
$entrypoint = "$PSScriptRoot\serve-local.py"
if ($details.ExecutablePath -ne "$PSScriptRoot\.venv\Scripts\python.exe" -or !$details.CommandLine.Contains($entrypoint)) { throw 'Unexpected service process identity.' }
$basePython = & "$PSScriptRoot\.venv\Scripts\python.exe" -c 'import sys; print(sys._base_executable)'
if ($LASTEXITCODE -ne 0 -or !$basePython) { throw 'Cannot verify the service child interpreter.' }
foreach ($child in (Get-CimInstance Win32_Process -Filter "ParentProcessId=$($state.pid)")) {
 if ($child.CommandLine -and $child.ExecutablePath -and $child.CommandLine.Contains($entrypoint) -and $child.ExecutablePath -eq $basePython) { Stop-Process -Id $child.ProcessId -ErrorAction SilentlyContinue }
}
if (!$p.HasExited) { Stop-Process -Id $p.Id -ErrorAction SilentlyContinue }
$p.WaitForExit(10000) | Out-Null
Remove-Item -LiteralPath $stateFile
Write-Output 'Managed Laya service stopped.'
