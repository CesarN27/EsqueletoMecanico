# Descarga los CSV del IMU desde el ESP32 y los guarda en la PC.
# Manda el comando "v", espera a que termine cada archivo y verifica el tamaño recibido.
# Antes de correrlo CIERRA el monitor serie (el puerto solo lo puede usar un programa).
#
# Uso:  powershell -ExecutionPolicy Bypass -File tools\descargar_imu.ps1 [-Puerto COM5] [-Carpeta datos]
param(
  [string]$Puerto = "COM5",
  [string]$Carpeta = "datos",
  [int]$TimeoutSeg = 180
)

$ErrorActionPreference = "Stop"
$puertoSerie = New-Object System.IO.Ports.SerialPort $Puerto, 115200, None, 8, one
$puertoSerie.ReadBufferSize = 4MB
$puertoSerie.Encoding = [System.Text.Encoding]::ASCII
# Con DTR/RTS en falso al abrir, la placa normalmente no se reinicia
$puertoSerie.DtrEnable = $false
$puertoSerie.RtsEnable = $false

try {
  $puertoSerie.Open()
} catch {
  Write-Host "No se pudo abrir $Puerto. Cierra el monitor serie (o cualquier programa que use el puerto) e intenta de nuevo." -ForegroundColor Red
  exit 1
}

try {
  $puertoSerie.DiscardInBuffer()
  $puertoSerie.Write("v`n")
  Write-Host "Descargando desde $Puerto (un archivo de 500 KB tarda ~45 s)..."

  $texto = New-Object System.Text.StringBuilder
  $reloj = [System.Diagnostics.Stopwatch]::StartNew()
  $ultimoDato = $reloj.Elapsed.TotalSeconds
  $completo = $false

  while ($reloj.Elapsed.TotalSeconds -lt $TimeoutSeg) {
    if ($puertoSerie.BytesToRead -gt 0) {
      [void]$texto.Append($puertoSerie.ReadExisting())
      $ultimoDato = $reloj.Elapsed.TotalSeconds
    } else {
      # Terminó cuando llega el mensaje que sigue al volcado
      if ($texto.ToString().Contains("Registro en pausa")) { $completo = $true; break }
      if ($reloj.Elapsed.TotalSeconds - $ultimoDato -gt 10) { break }  # 10 s sin datos
      Start-Sleep -Milliseconds 50
    }
  }

  $todo = $texto.ToString()
  if (-not $completo) {
    Write-Host "Aviso: la descarga no termino limpia (sin datos o tiempo agotado). Se guarda lo recibido." -ForegroundColor Yellow
  }

  New-Item -ItemType Directory -Force -Path $Carpeta | Out-Null
  $sello = Get-Date -Format "yyyyMMdd_HHmmss"
  $patron = '--- INICIO (\S+) \((\d+) bytes\) ---\r?\n(.*?)--- FIN \1 ---'
  $coincidencias = [regex]::Matches($todo, $patron, [System.Text.RegularExpressions.RegexOptions]::Singleline)

  if ($coincidencias.Count -eq 0) {
    Write-Host "No llego ningun archivo. Revisa que la placa tenga el firmware del IMU y que el monitor este cerrado." -ForegroundColor Red
    exit 1
  }

  $mejorFilas = -1
  $mejorDestino = $null

  foreach ($m in $coincidencias) {
    $rutaEsp = $m.Groups[1].Value
    $esperados = [int]$m.Groups[2].Value
    $contenido = $m.Groups[3].Value
    $recibidos = [System.Text.Encoding]::ASCII.GetByteCount($contenido)
    $filas = ([regex]::Matches($contenido, '(?m)^\d+,')).Count
    $nombre = ($rutaEsp.TrimStart('/') -replace '\.csv$', '') + "_$sello.csv"
    $destino = Join-Path $Carpeta $nombre
    [System.IO.File]::WriteAllText((Join-Path (Get-Location) $destino), $contenido, (New-Object System.Text.UTF8Encoding($false)))

    # Solo cuentan los archivos completos: uno cortado no debe ganar por tener más filas
    if ($recibidos -eq $esperados -and $filas -gt $mejorFilas) {
      $mejorFilas = $filas
      $mejorDestino = $destino
    }

    if ($recibidos -eq $esperados) {
      Write-Host ("OK     {0}: {1} filas, {2} bytes -> {3}" -f $rutaEsp, $filas, $recibidos, $destino) -ForegroundColor Green
    } else {
      Write-Host ("AVISO  {0}: llegaron {1} de {2} bytes (hay perdida) -> {3}" -f $rutaEsp, $recibidos, $esperados, $destino) -ForegroundColor Yellow
    }
  }

  # Copia con nombre fijo de la sesión con más datos, para leerla siempre desde la misma ruta
  if ($mejorDestino -and $mejorFilas -gt 0) {
    $ultimo = Join-Path $Carpeta "ultimo.csv"
    Copy-Item -Path $mejorDestino -Destination $ultimo -Force
    Write-Host ("ULTIMO {0} ({1} filas, copia de {2})" -f $ultimo, $mejorFilas, (Split-Path $mejorDestino -Leaf)) -ForegroundColor Cyan
  } else {
    Write-Host "Ninguna sesion tiene datos, no se actualizo ultimo.csv." -ForegroundColor Yellow
  }

  # El volcado deja el registro en pausa: se reanuda
  $puertoSerie.Write("r`n")
  Start-Sleep -Milliseconds 300
  Write-Host "Registro reanudado en el ESP32."
} finally {
  $puertoSerie.Close()
}
