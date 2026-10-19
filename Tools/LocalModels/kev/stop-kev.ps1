$ErrorActionPreference = 'Stop'
$stateFile = "$PSScriptRoot\service-process.json"
if (!(Test-Path $stateFile)) { Write-Output 'No managed Kev process recorded.'; exit 0 }
$state = Get-Content $stateFile -Raw | ConvertFrom-Json
$process = Get-Process -Id $state.pid -ErrorAction SilentlyContinue
if ($null -eq $process) { Remove-Item -LiteralPath $stateFile; Write-Output 'Kev is already stopped.'; exit 0 }
if ($process.StartTime.ToUniversalTime().Ticks.ToString() -ne $state.start_ticks) { throw 'PID was reused; refusing to stop an unrelated process.' }
$details = Get-CimInstance Win32_Process -Filter "ProcessId=$($state.pid)"
if ($details.CommandLine -notlike '*-m kev.serve*' -or $details.ExecutablePath -ne "$PSScriptRoot\.venv\Scripts\python.exe" -or !$state.run -or !$details.CommandLine.Contains($state.run)) { throw 'Process identity mismatch; refusing to stop it.' }
$basePython = & "$PSScriptRoot\.venv\Scripts\python.exe" -c 'import sys; print(sys._base_executable)'
if ($LASTEXITCODE -ne 0 -or !$basePython) { throw 'Cannot verify the service child interpreter.' }
# Windows venv launchers can host the actual interpreter as a child.
$children = Get-CimInstance Win32_Process -Filter "ParentProcessId=$($state.pid)"
foreach ($child in $children) {
    if ($child.CommandLine -and $child.ExecutablePath -and $child.CommandLine -like '*-m kev.serve*' -and $child.CommandLine.Contains($state.run) -and $child.ExecutablePath -eq $basePython) {
        Stop-Process -Id $child.ProcessId -ErrorAction SilentlyContinue
    }
}
if (!$process.HasExited) { Stop-Process -Id $state.pid -ErrorAction SilentlyContinue }
$process.WaitForExit(10000) | Out-Null
Remove-Item -LiteralPath $stateFile
Write-Output 'Managed Kev service stopped.'
