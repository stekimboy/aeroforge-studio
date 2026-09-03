# NOTE: the raw 1920x1080 SimScale captures this script reads (CFD_results/*/capture_*.png)
# are not in the public repo; the script is kept as the record of the crop boxes.
# Crops the raw 1920x1080 SimScale captures in CFD_results/ to the viewport
# (sidebar, toolbar and browser chrome removed; colour legends kept) and writes
# them to docs/media/ with descriptive names. Re-runnable.
Add-Type -AssemblyName System.Drawing
$root = Split-Path $PSScriptRoot -Parent
$src = Join-Path $root "CFD_results"
$out = Join-Path $root "docs\media"
New-Item -ItemType Directory -Force $out | Out-Null

# viewport of the SimScale post-processor at 1920x1080
$rect = New-Object System.Drawing.Rectangle 258, 202, 1418, 834

$picks = @(
    @{ in = "flying_wing\capture_02_iso_streamlines.png";                 out = "hero_flying_wing_streamlines.png" },
    @{ in = "flying_wing\capture_01_centreline_streamlines.png";          out = "flying_wing_centreline_streamlines.png" },
    @{ in = "flying_wing\capture_03_wing_streamlines_low.png";            out = "flying_wing_wing_streamlines.png" },
    @{ in = "flying_wing\capture_05_pressure_planform.png";               out = "flying_wing_pressure_planform.png" },
    @{ in = "flying_wing\capture_06_pressure_front.png";                  out = "flying_wing_pressure_front.png" },
    @{ in = "flying_wing\capture_07_pressure_side.png";                   out = "flying_wing_pressure_side.png" },
    @{ in = "tandem\capture_03_streamlines_iso.png";                      out = "tandem_streamlines_iso.png" },
    @{ in = "tandem\capture_02_streamlines_planform.png";                 out = "tandem_streamlines_planform.png" },
    @{ in = "tandem\capture_07_pressure_iso.png";                         out = "tandem_pressure_iso.png" },
    @{ in = "conventional\capture_01_centreline_streamlines_side.png";    out = "conventional_streamlines_side.png" },
    @{ in = "conventional\capture_05_wing_streamlines_iso.png";           out = "conventional_wing_streamlines.png" },
    @{ in = "conventional\capture_07_pressure_iso.png";                   out = "conventional_pressure_iso.png" },
    @{ in = "conventional\capture_08_pressure_front.png";                 out = "conventional_pressure_front.png" }
)

foreach ($p in $picks) {
    $img = [System.Drawing.Image]::FromFile((Join-Path $src $p.in))
    $bmp = New-Object System.Drawing.Bitmap $rect.Width, $rect.Height
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
    $g.DrawImage($img, (New-Object System.Drawing.Rectangle 0, 0, $rect.Width, $rect.Height), $rect, [System.Drawing.GraphicsUnit]::Pixel)
    $g.Dispose(); $img.Dispose()
    $dest = Join-Path $out $p.out
    $bmp.Save($dest, [System.Drawing.Imaging.ImageFormat]::Png)
    $bmp.Dispose()
    "{0,-45} {1,8:N0} bytes" -f $p.out, (Get-Item $dest).Length
}

# convergence plots are already clean - copy as-is
foreach ($c in @("flying_wing\flyingwing_force_convergence.jpg",
                 "tandem\tandem_force_convergence.jpg",
                 "conventional\conventional_force_convergence.jpg")) {
    Copy-Item (Join-Path $src $c) (Join-Path $out (Split-Path $c -Leaf)) -Force
}
"done"
