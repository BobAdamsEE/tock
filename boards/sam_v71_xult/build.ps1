<#
.SYNOPSIS
Build the SAMV71 Xplained Ultra Tock kernel and regenerate the flashable .bin.

.DESCRIPTION
`cargo build` only ever writes an ELF into ../../target/. The image that
flash_bootloader_kernel.jlink and flash_kernel.jlink actually load is
./sam_v71_xult.bin in this directory, produced by a separate objcopy step.
Running cargo directly refreshes the ELF and silently leaves that .bin stale.

That drift is invisible here for two reasons: the .bin is gitignored (*.bin in
tock's .gitignore), so git never reports it dirty; and it is always padded to
exactly 229376 bytes (the full 224 KB kernel region), so the size never changes
even when the contents do. Only the hash reveals it.

This script does both steps as one operation. Use it instead of
`cargo build --release`.

The objcopy flags match boards/Makefile.common (OBJCOPY_FLAGS) so this produces
a byte-identical image to `make`.

.EXAMPLE
.\build.ps1
#>
[CmdletBinding()]
param(
    # Skip the cargo build and only re-run objcopy on the existing ELF.
    [switch]$NoBuild
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$Platform = 'sam_v71_xult'
$Target   = 'thumbv7em-none-eabihf'
$BoardDir = $PSScriptRoot
# Tock builds into the workspace-root target/, two levels up from the board.
$Elf      = Join-Path $BoardDir "..\..\target\$Target\release\$Platform"
$Bin      = Join-Path $BoardDir "$Platform.bin"

Push-Location $BoardDir
try {
    # ---- 1. Build -----------------------------------------------------------
    if (-not $NoBuild) {
        Write-Host "  CARGO     building $Platform (release)" -ForegroundColor Cyan
        cargo build --release
        if ($LASTEXITCODE -ne 0) { throw "cargo build failed (exit $LASTEXITCODE)" }
    }

    if (-not (Test-Path $Elf)) { throw "ELF not found: $Elf" }

    # ---- 2. Locate llvm-objcopy --------------------------------------------
    # Shipped by the rustup `llvm-tools` component; path moves with the
    # toolchain, so discover it rather than hard-coding a version.
    $sysroot = (rustc --print sysroot).Trim()
    $objcopy = Get-ChildItem -Path (Join-Path $sysroot 'lib\rustlib') `
                             -Filter 'llvm-objcopy.exe' -Recurse -ErrorAction SilentlyContinue |
               Select-Object -First 1 -ExpandProperty FullName
    if (-not $objcopy) {
        throw "llvm-objcopy not found under $sysroot. Install it with: rustup component add llvm-tools"
    }

    # ---- 3. objcopy ---------------------------------------------------------
    # Flags mirror OBJCOPY_FLAGS in boards/Makefile.common:
    #   --strip-sections   keep the image from ballooning when SRAM is below flash
    #   --strip-all        drop non-allocated sections outside segments
    #   --remove-section .apps   .apps is an ELF-only placeholder for appended
    #                            apps; leaving it in would let the kernel image
    #                            overwrite installed applications
    $prevHash = if (Test-Path $Bin) { (Get-FileHash $Bin -Algorithm SHA256).Hash } else { $null }

    & $objcopy --output-target=binary --strip-sections --strip-all --remove-section .apps $Elf $Bin
    if ($LASTEXITCODE -ne 0) { throw "llvm-objcopy failed (exit $LASTEXITCODE)" }

    # ---- 4. Report ----------------------------------------------------------
    $size = (Get-Item $Bin).Length
    $hash = (Get-FileHash $Bin -Algorithm SHA256).Hash
    Write-Host "  BIN       $Platform.bin  $size bytes" -ForegroundColor Green
    Write-Host "  SHA256    $hash"

    if ($null -eq $prevHash) {
        Write-Host "  NOTE      .bin created (did not exist before)" -ForegroundColor Yellow
    } elseif ($prevHash -ne $hash) {
        Write-Host "  CHANGED   .bin differs from the previous build" -ForegroundColor Yellow
    } else {
        Write-Host "  UNCHANGED .bin is identical to the previous build"
    }

    Write-Host ""
    Write-Host "Flash kernel only:        JLink.exe -device ATSAMV71Q21B -if SWD -speed 4000 -CommandFile ..\..\..\flash_kernel.jlink"
    Write-Host "Flash bootloader+kernel:  JLink.exe -device ATSAMV71Q21B -if SWD -speed 4000 -CommandFile ..\..\..\flash_bootloader_kernel.jlink"
    Write-Host "(the latter also needs tock-bootloader\boards\samv71xplained-bootloader\build.ps1 to have been run)"
}
finally {
    Pop-Location
}
