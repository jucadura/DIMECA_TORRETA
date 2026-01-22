<template>
  <v-card class="mb-3">
  <!-- WRAPPER para poner canvas encima del stream -->
  <div class="cam-wrap" ref="wrapEl">
    <v-img
      :src="imgSrc"
      :aspect-ratio="camera.currentResolution.width / camera.currentResolution.height"
      cover
      :id="`cam-${props.camNum}-v-img`"
      crossorigin="anonymous"
      @error="onImageError"
    >
      <template #placeholder>
        <div class="d-flex align-center justify-center fill-height">
          <v-progress-circular color="grey-lighten-4" indeterminate />
        </div>
      </template>

      <template #error>
        <div class="d-flex align-center justify-center fill-height">
          <div style="height: 100px;" class="d-flex flex-column">
            <div class="my-auto" style="font-size: larger; font-weight: bold;">
              Something went wrong
            </div>
            <div class="mx-auto" style="font-size: smaller; opacity: 70%;">
              Retrying in 3 seconds...
            </div>
          </div>
        </div>
      </template>
    </v-img>

    <!-- CANVAS transparente encima -->
    <canvas ref="canvasEl" class="bbox-canvas"></canvas>
  </div>

  <!-- tu footer con botones queda igual -->
  <div class="d-flex justify-space-between align-center my-2 mx-3">
    <div>
      <div style="font-weight: bold; font-size: larger;">
        {{ camera.name && camera.name.length > 0 ? camera.name : `Camera #${camera.index}` }}
      </div>
      <div style="font-size: smaller; opacity: 70%;">
        {{ camera.currentImageFormatDescription }} @ {{ camera.currentFrameRate }} fps
        <span v-if="camera.currentQuality">
          (Quality: {{ camera.currentQuality }})
        </span>
      </div>
    </div>
    <div class="d-flex">
      <v-btn variant="tonal" @click="settingsDialog = true" :icon="mdiCog" class="mr-1" aria-label="Settings" />
      <v-btn variant="tonal" @click="captureFrame" :icon="mdiCameraOutline" class="mr-1" aria-label="Download Frame" />
      <v-btn variant="tonal" @click="captureRawFrame" :icon="mdiRaw" class="mr-1"
        aria-label="Download Raw Image (BIN)" />
    </div>
  </div>
</v-card>


  <v-dialog v-model="settingsDialog" max-width="500">
    <v-card>
      <v-card-title :prepend-icon="mdiCog">
        Camera Settings
      </v-card-title>
      <v-card-text>
        <v-select v-model="selectedImageFormatId" :items="camera.imageFormats" item-title="description" item-value="id"
          :disabled="settingsSaving" label="Image Format" />
        <v-slider v-model="selectedQuality" v-if="selectedFormat?.quality" :min="selectedFormat?.quality.min ?? 80"
          :max="selectedFormat?.quality.max ?? 95" :step="selectedFormat?.quality.step ?? 1" :disabled="settingsSaving"
          label="Quality">
          <template #append>
            {{ selectedQuality }}
          </template>
        </v-slider>
        <div v-else class="text-center mb-4">This image format may not support quality settings</div>
        <v-row>
          <v-col cols="9">
            <v-btn variant="tonal" @click="saveSettings" width="100%" :loading="settingsSaving">Save</v-btn>
          </v-col>
          <v-col cols="3">
            <v-btn variant="tonal" @click="settingsDialog = false" color="error" width="100%"
              :disabled="settingsSaving">Cancel</v-btn>
          </v-col>
        </v-row>
      </v-card-text>
    </v-card>
  </v-dialog>

  <v-snackbar v-model="saveStatusSnackbar" :timeout="2000" color="success">
    {{ saveStatusSnackbarText }}
  </v-snackbar>
</template>

<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted, nextTick } from 'vue'

import { mdiCameraOutline, mdiRaw, mdiCog } from '@mdi/js';
import { useMainStore } from '@/store/mainstore';

const LOADING_IMAGE_SRC = "/loading.jpg"

const mainStore = useMainStore()

const props = defineProps<{
  camNum: number,
}>()

const camera = computed(() => mainStore.clientCameras[props.camNum])

const imgSrc = ref<string>(LOADING_IMAGE_SRC)
// ===== Canvas overlay (boxes) =====
const wrapEl = ref<HTMLElement | null>(null)
const canvasEl = ref<HTMLCanvasElement | null>(null)
let pollId: ReturnType<typeof setInterval> | null = null

type PdBox = { x:number; y:number; w:number; h:number; score:number }
type PdResp = { count:number; frame_id:number; ts_ms:number; boxes: PdBox[] }

const lastPd = ref<PdResp | null>(null)

const resizeCanvas = () => {
  if (!wrapEl.value || !canvasEl.value) return
  const rect = wrapEl.value.getBoundingClientRect()
  canvasEl.value.width = Math.max(1, Math.round(rect.width))
  canvasEl.value.height = Math.max(1, Math.round(rect.height))
}

const drawBoxes = () => {
  const c = canvasEl.value
  if (!c) return
  const ctx = c.getContext('2d')
  if (!ctx) return

  ctx.clearRect(0, 0, c.width, c.height)

  const pd = lastPd.value
  if (!pd || !pd.boxes || pd.boxes.length === 0) return

  // Resolución ORIGINAL del frame (coords de /api/pd_last)
  const iw0 = camera.value.currentResolution.width
  const ih0 = camera.value.currentResolution.height

  const cw = c.width
  const ch = c.height

  // Ajuste para v-img cover (object-fit: cover)
  const scale = Math.max(cw / iw0, ch / ih0)
  const drawW = iw0 * scale
  const drawH = ih0 * scale
  const offX = (cw - drawW) / 2
  const offY = (ch - drawH) / 2

  ctx.lineWidth = 3
  ctx.strokeStyle = 'lime'
  ctx.fillStyle = 'lime'
  ctx.font = '16px sans-serif'

  for (const b of pd.boxes) {
    const x = b.x * scale + offX
    const y = b.y * scale + offY
    const w = b.w * scale
    const h = b.h * scale

    ctx.strokeRect(x, y, w, h)
    ctx.fillText(`${(b.score * 100).toFixed(1)}%`, x + 4, y + 18)
  }
}

const pollPd = async () => {
  try {
    const r = await fetch('/api/pd_last', { cache: 'no-store' })
    if (!r.ok) return
    lastPd.value = await r.json()
    drawBoxes()
  } catch {}
}

const startPdPolling = async () => {
  await nextTick()
  resizeCanvas()
  drawBoxes()

  if (pollId) clearInterval(pollId)
  pollId = setInterval(pollPd, 120) // ~8 fps
}

const stopPdPolling = () => {
  if (pollId) clearInterval(pollId)
  pollId = null
}

const settingsDialog = ref<boolean>(false)
const selectedImageFormatId = ref<number | string>(camera.value.currentImageFormat)
const selectedQuality = ref<number>(camera.value.currentQuality ?? 80)
const settingsSaving = ref<boolean>(false)
const saveStatusSnackbar = ref<boolean>(false)
const saveStatusSnackbarText = ref<string>("")
const retryTimeoutId = ref<ReturnType<typeof setTimeout> | null>(null)
const reloadTimeoutId = ref<ReturnType<typeof setTimeout> | null>(null)

const selectedFormat = computed(() => {
  return camera.value.imageFormats.find(format => format.id === selectedImageFormatId.value)
})

const onImageError = () => {
  if (imgSrc.value === LOADING_IMAGE_SRC) return;

  if (retryTimeoutId.value) {
    clearTimeout(retryTimeoutId.value)
  }

  retryTimeoutId.value = setTimeout(() => {
    reloadCameraSrc(1000)
    retryTimeoutId.value = null
  }, 3000)
}

const realCameraUrl = computed(() => {
  let port: number | null = null;
  let path: string | null = null;

  if (camera.value.src.startsWith(':')) {
    const [, portStr, pathStr] = camera.value.src.split(/[:\/]/);
    port = Number(portStr);
    path = pathStr;

    const realUrl = new URL(path, location.href);
    realUrl.port = port.toString();
    return realUrl.toString();
  } else {
    path = camera.value.src;
    return new URL(path, location.href).toString();
  }
})

const reloadCameraSrc = (ms: number = 100) => {
  imgSrc.value = LOADING_IMAGE_SRC;

  if (reloadTimeoutId.value) {
    clearTimeout(reloadTimeoutId.value)
    reloadTimeoutId.value = null
  }

  reloadTimeoutId.value = setTimeout(() => {
    imgSrc.value = realCameraUrl.value;
    reloadTimeoutId.value = null;
  }, ms);
}

const captureFrame = () => {
  const url = `/api/capture_image?source=${camera.value.index}`;

  const link = document.createElement('a');
  link.href = url;
  link.download = `camera_${camera.value.index}_image.jpg`;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

const captureRawFrame = () => {
  const url = `/api/capture_binary?source=${camera.value.index}`;

  const link = document.createElement('a');
  link.href = url;
  link.download = `camera_${camera.value.index}_raw.bin`;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

const saveSettings = async () => {
  imgSrc.value = LOADING_IMAGE_SRC;
  settingsSaving.value = true;
  await fetch('/api/set_camera_config', {
    method: 'POST',
    body: JSON.stringify({
      index: camera.value.index,
      image_format: selectedImageFormatId.value,
      jpeg_quality: selectedQuality.value
    })
  }).then(res => {
    if (res.ok) {
      saveStatusSnackbar.value = true;
      saveStatusSnackbarText.value = "Settings saved";
    } else {
      saveStatusSnackbar.value = true;
      saveStatusSnackbarText.value = "Failed to save settings";
    }
  }).finally(() => {
    settingsSaving.value = false;
    settingsDialog.value = false;
    reloadCameraSrc();
  })
}

watch(realCameraUrl, (newUrl) => {
  if (imgSrc.value !== LOADING_IMAGE_SRC) {
    imgSrc.value = newUrl;
  }
})

watch(selectedImageFormatId, () => {
  if (selectedFormat.value?.quality) {
    selectedQuality.value = selectedFormat.value?.quality.default ?? selectedFormat.value?.quality.max ?? 90;
  }
})

const onResize = () => { resizeCanvas(); drawBoxes() }

onMounted(() => {
  reloadCameraSrc()
  window.addEventListener('resize', onResize)
  startPdPolling()
})

onUnmounted(() => {
  window.removeEventListener('resize', onResize)
  stopPdPolling()

  if (retryTimeoutId.value) {
    clearTimeout(retryTimeoutId.value)
    retryTimeoutId.value = null
  }
})

</script>

<style scoped>
.cam-wrap{
  position: relative;
}
.bbox-canvas{
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}
</style>
