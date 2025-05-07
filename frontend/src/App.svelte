<script lang="ts">
  import { io } from "socket.io-client";
  import { onMount } from 'svelte';

onMount(() => {
  canvas.width = Math.round(canvas.clientWidth);
  canvas.height = Math.round(canvas.clientHeight);
  window.addEventListener("resize", ()=>{
    canvas.width = Math.round(canvas.clientWidth);
    canvas.height = Math.round(canvas.clientHeight);
  })
  const ctx = canvas.getContext('2d');
  let frame: number;

  (function loop() {
    frame = requestAnimationFrame(loop);

    if (ctx == null) {
      return;
    }

    ctx.fillStyle = "black";
    ctx.fillRect(0,0,canvas.width,canvas.height);
    ctx.fillStyle = "red";
    let center = [Math.round(canvas.width / 2), Math.round(canvas.height / 2)]
    ctx.strokeStyle = "lime";
    ctx.beginPath(); // Start a new path
    ctx.moveTo(center[0], center[1]);
    ctx.lineTo(center[0] - 200, center[1]);
    ctx.stroke(); // Render the path
    ctx.strokeStyle = "red";
    ctx.closePath();
    ctx.beginPath(); // Start a new path
    ctx.moveTo(center[0], center[1]);
    ctx.lineTo(center[0], center[1] - 200);
    ctx.stroke(); // Render the path
    ctx.closePath();

    for (let i = 0; i < pointCloud.length; i++) {
      let x = pointCloud[i][1] * -200 + center[0] - 1;
      let y = pointCloud[i][0] * -200 + center[1] - 1;
      // console.log(x);
      // console.log(y);
      ctx.fillRect(Math.round(x), Math.round(y), 2, 2);
    }
    ctx.beginPath(); // Start a new path
    ctx.arc(center[0], center[1], 0.127 * 200, 0, 2 * Math.PI);
    ctx.strokeStyle = "blue";
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.moveTo(center[0], center[1]);
    for (let i = 0; i < poseGraphNodes.length; i+=3) {
      ctx.lineTo(center[0] - 200 * poseGraphNodes[i+1], center[1] - 200 * poseGraphNodes[i]);
    }
    ctx.strokeStyle="white";
    ctx.stroke();
    ctx.closePath();

    ctx.fillStyle = "maroon";
    for (let i = 0; i < poseGraphNodes.length; i+=3) {
      let cloud = poseGraphScans.get(i);
      if (cloud == undefined) continue;

      let newCenter = [center[0] - 200 * poseGraphNodes[i+1], center[1] - 200 * poseGraphNodes[i]];
      let theta = poseGraphNodes[i+2];
      for (let i = 0; i < cloud.length; i++) {
        let x = cloud[i][1] * -200;
        let y = cloud[i][0] * -200;
        ctx.fillRect(Math.cos(theta) * x + Math.sin(theta) * y + newCenter[0] - 1, Math.cos(theta) * y - Math.sin(theta) * x + newCenter[1] - 1, 2, 2);
      }
    }
    
  })();

  return () => {
    cancelAnimationFrame(frame);
  };
});

let webSocketConnected: boolean = false;
let lidarConnected: boolean = false;
let arduinoConnected: boolean = false;
let pointCloud: number[][] = [];
let poseGraphNodes: number[] = [];
let canvas: HTMLCanvasElement;
let poseGraphScans: Map<number, number[][]> = new Map();

const socket = io();
socket.on("connect", () => {
  webSocketConnected = true;
  lidarConnected = false;
  arduinoConnected = false;
});
socket.on("disconnect", () => {
  webSocketConnected = false;
  lidarConnected = false;
  arduinoConnected = false;
});
socket.on("arduinoStatus", (status: boolean) => {
  arduinoConnected = status;
});
socket.on("lidarStatus", (status: boolean) => {
  lidarConnected = status;
});
socket.on("pointCloud", (c: any) => {
  pointCloud = c;
});
socket.on("newPoseGraphScan", (c: any) => {
  poseGraphScans.set(poseGraphNodes.length - 3, c);
});
socket.on("poseGraphNodes", (c: any) => {
  poseGraphNodes = c;
  console.log(c);
});

let keys = new Set();

document.addEventListener("keydown", (e)=> {
  keys.add(e.key);
  sendCommandedSpeeds();
})
document.addEventListener("keyup", (e)=> {
  keys.delete(e.key);
  sendCommandedSpeeds();
})

function sendCommandedSpeeds() {
  let vx = Number(keys.has("w")) - Number(keys.has("s"));
  let vy = 0.0;
  let omega = Number(keys.has("a")) - Number(keys.has("d"));
  omega *= 1.0;
  console.log([vx, vy, omega]);
  socket.emit("driveWithSpeeds", [vx, vy, omega]);
}
</script>

<div class="flex flex-col sm:flex-row">
<div class="flex flex-col p-4 bg-white dark:bg-slate-900">
  <h1 class="text-4xl font-extrabold leading-none tracking-tight mb-4 text-gray-900 md:text-5xl lg:text-6xl dark:text-white">Robot Interface</h1>
  {#if webSocketConnected}
    <span class="bg-green-100 text-green-800 text-xs mb-2 font-medium px-2.5 py-0.5 rounded-full dark:bg-green-900 dark:text-green-300">WebSocket connected</span>
  {:else}
  <span class="bg-red-100 text-red-800 text-xs mb-2 font-medium px-2.5 py-0.5 rounded-full dark:bg-red-900 dark:text-red-300">WebSocket disconnected</span>
  {/if}
  {#if lidarConnected}
    <span class="bg-green-100 text-green-800 text-xs mb-2 font-medium px-2.5 py-0.5 rounded-full dark:bg-green-900 dark:text-green-300">Lidar connected</span>
  {:else}
  <span class="bg-red-100 text-red-800 text-xs mb-2 font-medium px-2.5 py-0.5 rounded-full dark:bg-red-900 dark:text-red-300">Lidar disconnected</span>
  {/if}
  {#if arduinoConnected}
    <span class="bg-green-100 text-green-800 text-xs mb-2 font-medium px-2.5 py-0.5 rounded-full dark:bg-green-900 dark:text-green-300">Arduino connected</span>
  {:else}
  <span class="bg-red-100 text-red-800 text-xs mb-2 font-medium px-2.5 py-0.5 rounded-full dark:bg-red-900 dark:text-red-300">Arduino disconnected</span>
  {/if}
  <p class="text-sm text-gray-500 dark:text-gray-400 mb-2">Most recent scan had {pointCloud.length} points.</p>
  <p class="flex-grow"></p>
  <button type="button" class="text-white bg-blue-700 hover:bg-blue-800 focus:ring-4 focus:ring-blue-300 font-medium rounded-lg text-sm px-5 py-2.5 mb-2 dark:bg-blue-600 dark:hover:bg-blue-700 focus:outline-none dark:focus:ring-blue-800">Drive with WASD</button>
  <button type="button" class="text-white bg-blue-700 hover:bg-blue-800 focus:ring-4 focus:ring-blue-300 font-medium rounded-lg text-sm px-5 py-2.5 mb-2 dark:bg-blue-600 dark:hover:bg-blue-700 focus:outline-none dark:focus:ring-blue-800">Pathfind to location</button>
</div>
<div class="flex-grow">
  <canvas class="bg-black w-full h-screen" bind:this={canvas}></canvas>
</div>
</div>