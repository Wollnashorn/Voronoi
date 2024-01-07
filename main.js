let gl;
let pointShaderProgram;
let lineShaderProgram;
let redraw = true;

let camera = new Float32Array([-6.65580940246582, -48.12321472167969, 0.07580723613500595]);
let cameraZoomTarget = camera[2];
let lastTimestamp = 0;

const pointCount = 0;
let points = new Float32Array(2 * pointCount);
for (let i = 0; i < pointCount; ++i) {
    points[2 * i + 0] = 2.0 * Math.random() - 1;
    points[2 * i + 1] = 2.0 * Math.random() - 1;
}

const lineCount = 0;
let lines = new Float32Array(4 * lineCount);
for (let i = 0; i < lineCount; ++i) {
    lines[4 * i + 0] = 2.0 * Math.random() - 1;
    lines[4 * i + 1] = 2.0 * Math.random() - 1;
    lines[4 * i + 2] = 2.0 * Math.random() - 1;
    lines[4 * i + 3] = 2.0 * Math.random() - 1;
}

document.addEventListener("wheel", function(event) {
    event.preventDefault();
    cameraZoomTarget = cameraZoomTarget - event.deltaY * 0.001 * cameraZoomTarget;
    // window.requestAnimationFrame(draw);
    // window.setTimeout(draw, 100);
    redraw = true;
}, {passive: false});

document.addEventListener("touchmove", function(e) {
    e.preventDefault();
}, {passive: false});

document.addEventListener("touchstart", function(e) {
    e.preventDefault();
}, {passive: false});

document.addEventListener("DOMContentLoaded", function () {
    // Get the WebGL context
    const canvas = document.getElementById("webgl-canvas");

    canvas.style.cursor = 'grab';
    let pos = { top: 0, left: 0, x: 0, y: 0 };
    const evCache = [];
    let prevDiff = -1;

    const removeEvent = function(event) {
        // Remove this event from the target's cache
        const index = evCache.findIndex(
            (cachedEvent) => cachedEvent.pointerId === event.pointerId,
        );
        evCache.splice(index, 1);
    }

    const pointerDownHandler = function(event) {
        event.preventDefault();
        evCache.push(event);

        canvas.style.cursor = 'grabbing';

        pos = {
            x: event.clientX,
            y: event.clientY,
        };
    
        document.addEventListener('pointermove', pointerMoveHandler);
    };
    
    const pointerMoveHandler = function(event) {
        event.preventDefault();

        // Find this event in the cache and update its record with this event
        const index = evCache.findIndex(
            (cachedEv) => cachedEv.pointerId === event.pointerId,
        );
        evCache[index] = event;

        // If two pointers are down, check for pinch gestures
        if (evCache.length === 2) {
            // Calculate the distance between the two pointers
            let curDiff = Math.hypot(evCache[0].clientX - evCache[1].clientX, evCache[0].clientY - evCache[1].clientY);
            curDiff /= Math.min(canvas.clientWidth, canvas.clientHeight);

            if (prevDiff > 0) {
                const delta = curDiff - prevDiff;
                cameraZoomTarget = cameraZoomTarget + delta * 1.0 * cameraZoomTarget;
                redraw = true;
            }

            // Cache the distance for the next move event
            prevDiff = curDiff;
        } else {
            const dx = event.clientX - pos.x;
            const dy = event.clientY - pos.y;
        
            camera[0] += window.devicePixelRatio * 2.0 / (camera[2] * canvas.width) * dx;
            camera[1] -= window.devicePixelRatio * 2.0 / (camera[2] * canvas.width) * dy;
            // draw();
            redraw = true;
            // window.requestAnimationFrame(draw);
            // window.setTimeout(draw, 100);
    
            pos.x = event.clientX;
            pos.y = event.clientY;
        }
    };
    
    const pointerUpHandler = function(event) {
        canvas.style.cursor = 'grab';
        // canvas.style.removeProperty('user-scanvasct');
    
        removeEvent(event);
        if (evCache.length < 2) prevDiff = -1;

        document.removeEventListener('pointermove', pointerMoveHandler);
    };
    
    canvas.addEventListener('pointerdown', pointerDownHandler);
    document.addEventListener('pointerup', pointerUpHandler);


    // gl = canvas.getContext("webgl", {antialias: true});
    gl = canvas.getContext("webgl", {alpha: false, antialias: false});

    if (!gl) {
        console.error("Unable to initialize WebGL. Your browser may not support it.");
        return;
    }

    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

    // Vertex and fragment shader code for points
    const pointVertexShaderSource = `
        precision mediump float;
        attribute vec2 attribPosition;
        attribute vec2 attribUV;
        uniform vec3 camera;
        uniform vec2 resolution;
        varying vec2 uv;
        void main() {
            float radius = max(20.0 / 720.0 * camera.z, 2.5 / resolution.x);
            gl_Position = vec4(attribUV * radius + camera.z * (camera.xy + attribPosition), 0.0, 1.0);
            gl_Position.y *= resolution.x / resolution.y;
            uv = attribUV;
        }
    `;

    const pointFragmentShaderSource = `
        precision mediump float;
        uniform vec3 camera;
        uniform vec2 resolution;
        varying vec2 uv;
        void main() {
            // if (length(uv) > 0.5)
            //     discard;
            float smoothing = (72.0 / resolution.x) / camera.z;
            float alpha = smoothstep(0.5 + smoothing, 0.5 - smoothing, length(uv));
            alpha *= sqrt(min(20.0 * camera.z / 2.5, 1.0));
            gl_FragColor = vec4(1.0, 1.0, 1.0, alpha);
        }
    `;

    // Vertex and fragment shader code for lines
    const lineVertexShaderSource = `
        precision mediump float;
        attribute vec2 attribPosition;
        attribute vec2 attribUV;
        attribute vec2 attribNormal;
        uniform vec3 camera;
        uniform vec2 resolution;
        varying vec2 uv;
        varying vec2 normal;
        void main() {
            float width = max(10.0 / 720.0 * camera.z, 1.5 / resolution.x);
            // gl_Position = vec4(camera.xy + attribNormal * width + camera.z * (attribPosition), 0.0, 1.0);
            gl_Position = vec4(attribNormal * width + camera.z * (camera.xy + attribPosition), 0.0, 1.0);
            gl_Position.y *= resolution.x / resolution.y;
            uv = attribUV;
            normal = attribNormal;
        }
    `;

    const lineFragmentShaderSource = `
        precision mediump float;
        uniform vec3 camera;
        uniform vec2 resolution;
        varying vec2 uv;
        varying vec2 normal;
        void main() {
            // if (uv.x < 0.1 || uv.x > 0.9 || uv.y < 0.1 || uv.y > 0.9)
            //     discard;
            // gl_FragColor = vec4(uv.y, 1.0, 0.0, 1.0); // Green color for lines

            // float smoothing = 0.15 / (camera.z + camera.z * camera.z);
            // float smoothing = 0.15 / (camera.z * camera.z);
            // float smoothing = pow(0.4 / camera.z, 1.4);

            // float smoothing = 0.15 / camera.z;
            // float alpha = smoothstep(0.5 + smoothing, 0.5 - smoothing, 6.0 * (1.0 + 0.25 * camera.z) * abs(uv.y - 0.5));
            // alpha *= pow(camera.z, 0.05);
            // // float alpha = step(0.5, 16.0 * abs(uv.y - 0.5));
            // gl_FragColor = vec4(94.0 / 255.0, 209.0 / 255.0, 125.0 / 255.0, alpha);

            float smoothing = (144.0 / resolution.x) / (camera.z > 1.0 ? camera.z : (camera.z * camera.z + camera.z));
            // smoothing = sqrt(smoothing);
            float alpha = smoothstep(0.5 + smoothing, 0.5 - smoothing, (0.25 * smoothing + 2.0) * abs(uv.y - 0.5));
            alpha *= sqrt(min(10.0 * camera.z / 1.5, 1.0));
            // gl_FragColor = vec4(94.0 / 255.0, 209.0 / 255.0, 125.0 / 255.0, alpha);
            gl_FragColor = vec4(120.0 / 255.0, 209.0 / 255.0, 125.0 / 255.0, alpha);
            //gl_FragColor = vec4(94.0 / 255.0, 209.0 / 255.0, 125.0 / 255.0, 1.0);
            // gl_FragColor = vec4(normal * 0.5 + 0.5, 0.0, alpha);

            // if (camera.z < 1.25 / 10.0) gl_FragColor = vec4(10.0 * camera.z / 1.25, 0.0, 0.0, alpha);
        }
    `;

    // Compile shaders
    function compileShader(type, source) {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);

        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            console.error(gl.getShaderInfoLog(shader));
            gl.deleteShader(shader);
            return null;
        }

        return shader;
    }

    const pointVertexShader = compileShader(gl.VERTEX_SHADER, pointVertexShaderSource);
    const pointFragmentShader = compileShader(gl.FRAGMENT_SHADER, pointFragmentShaderSource);

    const lineVertexShader = compileShader(gl.VERTEX_SHADER, lineVertexShaderSource);
    const lineFragmentShader = compileShader(gl.FRAGMENT_SHADER, lineFragmentShaderSource);

    // Create and link the shader programs
    pointShaderProgram = gl.createProgram();
    gl.attachShader(pointShaderProgram, pointVertexShader);
    gl.attachShader(pointShaderProgram, pointFragmentShader);
    gl.linkProgram(pointShaderProgram);

    lineShaderProgram = gl.createProgram();
    gl.attachShader(lineShaderProgram, lineVertexShader);
    gl.attachShader(lineShaderProgram, lineFragmentShader);
    gl.linkProgram(lineShaderProgram);

    if (!gl.getProgramParameter(pointShaderProgram, gl.LINK_STATUS) || !gl.getProgramParameter(lineShaderProgram, gl.LINK_STATUS)) {
        console.error("Shader program linking failed");
        console.error(gl.getProgramInfoLog(pointShaderProgram));
        return;
    }
    
    let loadedPoints = false, loadedLines = false;
    fetch("diagram_europe_points.bin")
    // fetch("diagram_dense_points.bin")
        .then(response => response.arrayBuffer())
        .then(buffer => {
            const float32Array = new Float32Array(buffer);
            console.log("Loaded " + float32Array.length / 2 + " points");
            // Use the float32Array as needed
            points = float32Array;
            loadedPoints = true;
            if (loadedPoints && loadedLines) generateBuffers();
        })
        .catch(error => console.error("Error fetching the file:", error));
    
    fetch("diagram_europe_lines.bin")
    // fetch("diagram_dense_lines.bin")
        .then(response => response.arrayBuffer())
        .then(buffer => {
            const float32Array = new Float32Array(buffer);
            console.log("Loaded " + float32Array.length / 4 + " lines");
            // Use the float32Array as needed
            lines = float32Array;
            loadedLines = true;
            if (loadedPoints && loadedLines) generateBuffers();
        })
        .catch(error => console.error("Error fetching the file:", error));

    // draw();
    window.requestAnimationFrame(draw);
});

let pointVertexBuffer;
let pointTexcoordBuffer;
let lineVertexBuffer;
let lineTexcoordBuffer;
let lineNormalBuffer;

function generateBuffers() {
    const triangle = new Float32Array([
        Math.sin(0), Math.cos(0),
        Math.sin(2 * Math.PI / 3), Math.cos(2 * Math.PI / 3),
        Math.sin(4 * Math.PI / 3), Math.cos(4 * Math.PI / 3)
    ]);

    const radius = 0.1;
    let pointVertices = new Float32Array(6 * points.length / 2);
    let pointTexcoords = new Float32Array(6 * points.length / 2);
    for (let i = 0; i < points.length / 2; ++i) {
        pointVertices.set(triangle, i * 6);
        pointTexcoords.set(triangle, i * 6);
        // for (let v = i * 6; v < i * 6 + 6; ++v) {
        //     pointVertices[v] = points[2 * i + v % 2] + radius * pointVertices[v];
        // }
        for (let v = i * 6; v < i * 6 + 6; ++v) {
            pointVertices[v] = points[2 * i + v % 2];
        }
    }

    pointVertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, pointVertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, pointVertices, gl.STATIC_DRAW);
    pointTexcoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, pointTexcoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, pointTexcoords, gl.STATIC_DRAW);

    const width = 0.04;
    let lineVertices = new Float32Array(12 * lines.length / 4);
    let lineTexcoords = new Float32Array(12 * lines.length / 4);
    let lineNormals = new Float32Array(12 * lines.length / 4);
    for (let i = 0; i < lines.length / 4; ++i) {
        // Calculate a billboard quad (4 vec2 positions + 4 vec2 texcoords)
        const dx = lines[4 * i + 2] - lines[4 * i + 0];
        const dy = lines[4 * i + 3] - lines[4 * i + 1];        
        const length = Math.sqrt(dx * dx + dy * dy);
        const nx = -dy / length;
        const ny = dx / length;

        // Quad vertices
        lineVertices.set([
            lines[4 * i + 0], lines[4 * i + 1],
            lines[4 * i + 0], lines[4 * i + 1],
            lines[4 * i + 2], lines[4 * i + 3],

            lines[4 * i + 2], lines[4 * i + 3],
            lines[4 * i + 2], lines[4 * i + 3],
            lines[4 * i + 0], lines[4 * i + 1]
        ], i * 12);

        // Texcoords
        lineTexcoords.set([
            0.0, 1.0,
            0.0, 0.0,
            1.0, 1.0,

            1.0, 1.0,
            1.0, 0.0,
            0.0, 0.0
        ], i * 12);

        lineNormals.set([
            +nx, +ny,
            -nx, -ny,
            +nx, +ny,

            +nx, +ny,
            -nx, -ny,
            -nx, -ny
        ], i * 12);
    }

    lineVertexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, lineVertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, lineVertices, gl.STATIC_DRAW);
    lineTexcoordBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, lineTexcoordBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, lineTexcoords, gl.STATIC_DRAW);
    lineNormalBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, lineNormalBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, lineNormals, gl.STATIC_DRAW);

    redraw = true;
}

function draw(timestamp) {
    const dt = (timestamp - lastTimestamp);
    lastTimestamp = timestamp;

    if (!redraw) {
        window.requestAnimationFrame(draw);
        return;
    }

    console.log(dt);

    const zoomDelta = cameraZoomTarget - camera[2];
    camera[2] = camera[2] + (dt / 144.0) * zoomDelta;

    if (Math.abs(zoomDelta) < 0.00001) redraw = false;

    const canvas = document.getElementById("webgl-canvas");

    gl.clearColor(0.118, 0.118, 0.118, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT);

    const resolution = new Float32Array([gl.canvas.width, gl.canvas.height]);
    gl.viewport(0, 0, canvas.width, canvas.height);

    // const camera = new Float32Array([0.0, 0.0, 0.09]);
    // const camera = new Float32Array([0.0, 0.0, 1.0]);
    // const camera = new Float32Array([0.0, 0.0, 0.04]);

    // const adjustedCamera = new Float32Array(camera);
    // adjustedCamera[2] *= 720.0 / resolution[0];

    // Render lines using the line shader program
    gl.useProgram(lineShaderProgram);
    gl.uniform3fv(gl.getUniformLocation(lineShaderProgram, "camera"), camera);
    gl.uniform2fv(gl.getUniformLocation(lineShaderProgram, "resolution"), resolution);

    gl.bindBuffer(gl.ARRAY_BUFFER, lineVertexBuffer);
    const linePositionAttrib = gl.getAttribLocation(lineShaderProgram, "attribPosition");
    gl.vertexAttribPointer(linePositionAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(linePositionAttrib);

    gl.bindBuffer(gl.ARRAY_BUFFER, lineTexcoordBuffer);
    const lineUVAttrib = gl.getAttribLocation(lineShaderProgram, "attribUV");
    gl.vertexAttribPointer(lineUVAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(lineUVAttrib);

    gl.bindBuffer(gl.ARRAY_BUFFER, lineNormalBuffer);
    const lineNormalAttrib = gl.getAttribLocation(lineShaderProgram, "attribNormal");
    gl.vertexAttribPointer(lineNormalAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(lineNormalAttrib);

    gl.drawArrays(gl.TRIANGLES, 0, 6 * lines.length / 4);


    // Render points using the point shader program
    gl.useProgram(pointShaderProgram);
    gl.uniform3fv(gl.getUniformLocation(pointShaderProgram, "camera"), camera);
    gl.uniform2fv(gl.getUniformLocation(pointShaderProgram, "resolution"), resolution);

    gl.bindBuffer(gl.ARRAY_BUFFER, pointVertexBuffer);
    const pointPositionAttrib = gl.getAttribLocation(pointShaderProgram, "attribPosition");
    gl.vertexAttribPointer(pointPositionAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(pointPositionAttrib);

    gl.bindBuffer(gl.ARRAY_BUFFER, pointTexcoordBuffer);
    const pointUVAttrib = gl.getAttribLocation(pointShaderProgram, "attribUV");
    gl.vertexAttribPointer(pointUVAttrib, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(pointUVAttrib);

    gl.drawArrays(gl.TRIANGLES, 0, 3 * points.length / 2);

    // redraw = false;
    // window.setTimeout(function() { redraw = false; }, 100);

    window.requestAnimationFrame(draw);
}

let canvasSizeInitialized = false;
function resizeCanvas() {
    const canvas = document.getElementById('webgl-canvas');

    devicePixelRatio = window.devicePixelRatio; // / window.visualViewport.scale;
    const containerWidth = canvas.clientWidth;
    const containerHeight = canvas.clientHeight;
    // let width, height;
    // if (containerWidth / aspectRatio > containerHeight) {
    //     // The container's width is the limiting dimension
    //     width = containerHeight * aspectRatio;
    //     height = containerHeight;
    // } else {
    //     // The container's height is the limiting dimension
    //     width = containerWidth;
    //     height = containerWidth / aspectRatio;
    // }

    const scaleRatio = canvas.width / (containerWidth * devicePixelRatio);

    canvas.width = containerWidth * devicePixelRatio;
    canvas.height = containerHeight * devicePixelRatio;
    // canvas.width = containerWidth;
    // canvas.height = containerHeight;
    redraw = true;
    // draw();

    if (canvasSizeInitialized) {
        camera[2] *= scaleRatio;
        cameraZoomTarget *= scaleRatio;
    }
    canvasSizeInitialized = true;
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();
