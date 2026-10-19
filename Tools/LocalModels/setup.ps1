param(
    [ValidateSet('laya', 'kev', 'both')][string]$Backend = 'both',
    [string]$DeploymentRoot = 'D:\AI',
    [string]$ProjectPath = (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent),
    [string]$Python = 'python',
    [string]$UvPath = '',
    [switch]$VerifyOnly,
    [switch]$Plan
)
$ErrorActionPreference = 'Stop'
$arguments = @("$PSScriptRoot\setup.py", '--backend', $Backend,
    '--deployment-root', $DeploymentRoot, '--project', $ProjectPath)
if ($UvPath) { $arguments += @('--uv', $UvPath) }
if ($VerifyOnly) { $arguments += '--verify-only' }
if ($Plan) { $arguments += '--plan' }
& $Python @arguments
exit $LASTEXITCODE
