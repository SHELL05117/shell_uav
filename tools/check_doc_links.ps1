<#
.SYNOPSIS
    Check relative links and in-page anchors of Markdown / HTML documents in this project.

.DESCRIPTION
    Scans every .md / .html file in the workspace and checks:
      1. relative link targets exist on disk;
      2. when a target is a Markdown file with a #fragment, that fragment can be
         produced from one of its headings (GitHub style slug);
      3. same-file fragments (#xxx) can be produced from the current file headings.
    http(s), mailto, Windows absolute paths (D:\...), UNC paths and root paths
    (/...) are listed separately as external references for manual review.

    Read-only: nothing is modified. Kept ASCII-only on purpose so Windows
    PowerShell 5.1 parses it regardless of the file encoding.

.PARAMETER Root
    Project root. Defaults to the parent directory of the script folder.

.PARAMETER OutJson
    Result JSON path. Defaults to check_doc_links_result.json in TEMP.

.EXAMPLE
    powershell -NoProfile -File tools/check_doc_links.ps1
#>
[CmdletBinding()]
param(
    [string]$Root = '',
    [string]$OutJson = (Join-Path ([System.IO.Path]::GetTempPath()) 'check_doc_links_result.json')
)

$ErrorActionPreference = 'Stop'
if (-not $Root) { $Root = Split-Path -Parent $PSScriptRoot }
$Root = (Resolve-Path -LiteralPath $Root).Path

$excludeSegments = @('/.git/', '/build/', '/devel/', '/install/', '/node_modules/', '/__pycache__/',
    '/temp1022/', '/temp1025/', '/tmp/', '/recordings/', '/.venv/', '/venv/', '/.catkin_tools/')

function Get-Slug([string]$heading) {
    $t = $heading.ToLowerInvariant()
    $t = [regex]::Replace($t, '[^\p{L}\p{N}\p{M} _\-]', '')
    $t = $t.Trim() -replace ' ', '-'
    return $t
}

function Get-AnchorSet([string[]]$lines, [bool]$isHtml = $false) {
    $set = New-Object 'System.Collections.Generic.HashSet[string]'
    $seen = @{}
    if ($isHtml) {
        foreach ($line in $lines) {
            foreach ($m in [regex]::Matches($line, '\b(?:id|name)\s*=\s*"([^"]+)"')) { [void]$set.Add($m.Groups[1].Value) }
        }
        return , $set
    }
    foreach ($line in $lines) {
        if ($line -match '^\s{0,3}(#{1,6})\s+(.+?)\s*#*\s*$') {
            $slug = Get-Slug $Matches[2]
            if ([string]::IsNullOrEmpty($slug)) { continue }
            if ($seen.ContainsKey($slug)) {
                $seen[$slug]++
                [void]$set.Add(("{0}-{1}" -f $slug, ($seen[$slug] - 1)))
            }
            else {
                $seen[$slug] = 0
                [void]$set.Add($slug)
            }
        }
    }
    return , $set
}

$files = Get-ChildItem -Path $Root -Recurse -File -Include *.md, *.html |
    Where-Object {
        $norm = $_.FullName -replace '\\', '/'
        -not ($excludeSegments | Where-Object { $norm.Contains($_) })
    } | Sort-Object FullName

$anchorCache = @{}
$missingFiles = New-Object System.Collections.Generic.List[object]
$missingAnchors = New-Object System.Collections.Generic.List[object]
$external = New-Object System.Collections.Generic.List[object]
$linkCount = 0
$fileCount = 0

foreach ($file in $files) {
    $fileCount++
    $relFile = $file.FullName.Substring($Root.Length + 1) -replace '\\', '/'
    $lines = [System.IO.File]::ReadAllLines($file.FullName)
    $isHtml = $file.Extension -eq '.html'
    $pattern = if ($isHtml) { '\bhref\s*=\s*"([^"]+)"' } else { '!?\[[^\]]*\]\(\s*(<[^>]+>|[^)\s]+)(?:\s+"[^"]*")?\s*\)' }

    for ($i = 0; $i -lt $lines.Count; $i++) {
        foreach ($m in [regex]::Matches($lines[$i], $pattern)) {
            $target = $m.Groups[1].Value.Trim()
            if ($target.StartsWith('<') -and $target.EndsWith('>')) { $target = $target.Substring(1, $target.Length - 2) }
            if ([string]::IsNullOrWhiteSpace($target)) { continue }
            if ($target -match '^(?i)(https?:|mailto:|ftp:|tel:|data:|javascript:)') { continue }
            if ($target -match '^[A-Za-z]:') { $external.Add([pscustomobject]@{ file = $relFile; line = $i + 1; target = $target; reason = 'windows-absolute' }); continue }
            if ($target -match '^(\\\\|/)') { $external.Add([pscustomobject]@{ file = $relFile; line = $i + 1; target = $target; reason = 'unc-or-root' }); continue }

            $linkCount++
            $decoded = $target
            try { $decoded = [System.Uri]::UnescapeDataString($target) } catch { }
            $hash = $decoded.IndexOf('#')
            $pathPart = if ($hash -ge 0) { $decoded.Substring(0, $hash) } else { $decoded }
            $anchorPart = if ($hash -ge 0) { $decoded.Substring($hash + 1) } else { '' }
            $anchorPart = $anchorPart.Trim().ToLowerInvariant()

            if ($pathPart -eq '') { $resolved = $file.FullName }
            else { $resolved = [System.IO.Path]::GetFullPath((Join-Path $file.DirectoryName $pathPart)) }

            if (-not (Test-Path -LiteralPath $resolved)) {
                $missingFiles.Add([pscustomobject]@{ file = $relFile; line = $i + 1; target = $target; resolved = $resolved })
                continue
            }

            if ($anchorPart -eq '') { continue }
            # GitHub line anchors (#L11, #L11-L20) address lines, not headings
            if ($anchorPart -match '^l\d+(-l\d+)?$') { continue }
            if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) { continue }

            if (-not $anchorCache.ContainsKey($resolved)) {
                $targetIsHtml = [System.IO.Path]::GetExtension($resolved) -eq '.html'
                $anchorCache[$resolved] = Get-AnchorSet ([System.IO.File]::ReadAllLines($resolved)) $targetIsHtml
            }
            if (-not $anchorCache[$resolved].Contains($anchorPart)) {
                $missingAnchors.Add([pscustomobject]@{ file = $relFile; line = $i + 1; target = $target; anchor = $anchorPart })
            }
        }
    }
}

# Known historical misses: pre-2026-10-22 workspace files that were never migrated
# into this repository. Exact file+target pairs only, so new breakage still fails.
$knownMissing = New-Object System.Collections.Generic.List[object]
$unexpected = New-Object System.Collections.Generic.List[object]
$allowFile = Join-Path $Root 'tools/doc_links_known_missing.json'
$allowKeys = @{}
if (Test-Path -LiteralPath $allowFile) {
    $allow = Get-Content -LiteralPath $allowFile -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($e in $allow.entries) { $allowKeys["$($e.file)|$($e.target)"] = $e.reason }
}
foreach ($m in $missingFiles) {
    $key = "$($m.file)|$($m.target)"
    if ($allowKeys.ContainsKey($key)) {
        $knownMissing.Add([pscustomobject]@{ file = $m.file; line = $m.line; target = $m.target; reason = $allowKeys[$key] })
    }
    else { $unexpected.Add($m) }
}

$result = [pscustomobject]@{
    root           = $Root
    filesScanned   = $fileCount
    linksChecked   = $linkCount
    missingFiles   = $unexpected
    missingAnchors = $missingAnchors
    knownMissing   = $knownMissing
    externalRefs   = $external
}

$result | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $OutJson -Encoding UTF8

Write-Host ("ROOT={0}" -f $Root)
Write-Host ("FILES={0} LINKS={1} MISSING_FILE={2} MISSING_ANCHOR={3} KNOWN_MISSING={4} EXTERNAL={5}" -f `
        $fileCount, $linkCount, $unexpected.Count, $missingAnchors.Count, $knownMissing.Count, $external.Count)
Write-Host ("RESULT_JSON={0}" -f $OutJson)

if ($unexpected.Count -gt 0) {
    Write-Host ''
    Write-Host '[MISSING FILE]'
    $unexpected | ForEach-Object { Write-Host ("  {0}:{1}  -> {2}" -f $_.file, $_.line, $_.target) }
}
if ($missingAnchors.Count -gt 0) {
    Write-Host ''
    Write-Host '[MISSING ANCHOR]'
    $missingAnchors | ForEach-Object { Write-Host ("  {0}:{1}  -> {2}" -f $_.file, $_.line, $_.target) }
}
if ($knownMissing.Count -gt 0) {
    Write-Host ''
    Write-Host '[KNOWN HISTORICAL MISS - allowlisted in tools/doc_links_known_missing.json]'
    $knownMissing | ForEach-Object { Write-Host ("  {0}:{1}  -> {2}" -f $_.file, $_.line, $_.target) }
}
if ($external.Count -gt 0) {
    Write-Host ''
    Write-Host '[EXTERNAL / ABSOLUTE - manual review]'
    $external | ForEach-Object { Write-Host ("  {0}:{1}  -> {2}" -f $_.file, $_.line, $_.target) }
}

if ($unexpected.Count -gt 0 -or $missingAnchors.Count -gt 0) { exit 1 }
exit 0
