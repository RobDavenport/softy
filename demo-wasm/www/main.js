import init, { SpringsDemo, RopeDemo, ClothDemo, SoftBodyDemo } from '../pkg/softy_demo.js';

let canvas, ctx;
let currentTab = 'springs';
let demos = {};
let mouseX = 300, mouseY = 300;
let mouseDown = false;
let lastTime = 0;
let windStrength = 0;

async function main() {
    await init();

    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');

    demos.springs = new SpringsDemo();
    demos.rope = new RopeDemo(20);
    demos.cloth = new ClothDemo(15, 12, 25.0);
    demos.softbody = new SoftBodyDemo();

    // Tab switching
    document.querySelectorAll('.tab').forEach(function(tab) {
        tab.addEventListener('click', function() {
            document.querySelector('.tab.active').classList.remove('active');
            tab.classList.add('active');
            currentTab = tab.dataset.tab;
            updateControls();
            updateInfo();
        });
    });

    // Mouse tracking
    canvas.addEventListener('mousemove', function(e) {
        var rect = canvas.getBoundingClientRect();
        var scaleX = canvas.width / rect.width;
        var scaleY = canvas.height / rect.height;
        mouseX = (e.clientX - rect.left) * scaleX;
        mouseY = (e.clientY - rect.top) * scaleY;
    });

    canvas.addEventListener('mousedown', function() {
        mouseDown = true;
    });

    canvas.addEventListener('mouseup', function() {
        mouseDown = false;
    });

    canvas.addEventListener('mouseleave', function() {
        mouseDown = false;
    });

    canvas.addEventListener('click', function(e) {
        var rect = canvas.getBoundingClientRect();
        var scaleX = canvas.width / rect.width;
        var scaleY = canvas.height / rect.height;
        var x = (e.clientX - rect.left) * scaleX;
        var y = (e.clientY - rect.top) * scaleY;

        if (currentTab === 'cloth') {
            // Find nearest grid position and tear
            var cols = demos.cloth.cols();
            var rows = demos.cloth.rows();
            var pos = demos.cloth.positions();
            var nearest = 0;
            var nearestDist = Infinity;
            for (var i = 0; i < cols * rows; i++) {
                var px = pos[i * 2];
                var py = pos[i * 2 + 1];
                var d = (px - x) * (px - x) + (py - y) * (py - y);
                if (d < nearestDist) {
                    nearestDist = d;
                    nearest = i;
                }
            }
            var col = nearest % cols;
            var row = Math.floor(nearest / cols);
            if (row > 0) {
                demos.cloth.tear_at(col, row);
            }
        } else if (currentTab === 'softbody') {
            demos.softbody.poke(x, y, (x - 350) * -0.5, -80.0);
        }
    });

    updateControls();
    updateInfo();
    lastTime = performance.now();
    requestAnimationFrame(loop);
}

function loop(timestamp) {
    var dt = Math.min((timestamp - lastTime) / 1000, 1 / 30);
    lastTime = timestamp;

    update(dt);
    render();
    requestAnimationFrame(loop);
}

function update(dt) {
    if (currentTab === 'springs') {
        demos.springs.set_target(mouseX, mouseY);
        demos.springs.update(dt);
    } else if (currentTab === 'rope') {
        demos.rope.move_pin(mouseX, mouseY);
        demos.rope.update(dt);
    } else if (currentTab === 'cloth') {
        if (windStrength !== 0) {
            demos.cloth.apply_wind(windStrength);
        }
        demos.cloth.update(dt);
    } else if (currentTab === 'softbody') {
        demos.softbody.update(dt);
    }
}

function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (currentTab === 'springs') renderSprings();
    else if (currentTab === 'rope') renderRope();
    else if (currentTab === 'cloth') renderCloth();
    else if (currentTab === 'softbody') renderSoftBody();
}

function renderSprings() {
    var pos = demos.springs.positions();
    var colors = ['#7fdbca', '#ff6b6b', '#ffd93d'];
    var labels = ['Critical', 'Underdamped', 'Overdamped'];

    // Draw target crosshair
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(mouseX - 15, mouseY);
    ctx.lineTo(mouseX + 15, mouseY);
    ctx.moveTo(mouseX, mouseY - 15);
    ctx.lineTo(mouseX, mouseY + 15);
    ctx.stroke();

    // Draw target circle
    ctx.strokeStyle = '#555';
    ctx.beginPath();
    ctx.arc(mouseX, mouseY, 20, 0, Math.PI * 2);
    ctx.stroke();

    for (var i = 0; i < 3; i++) {
        var x = pos[i * 2];
        var y = pos[i * 2 + 1];

        // Draw line from ball to target
        ctx.strokeStyle = colors[i] + '40';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(mouseX, mouseY);
        ctx.stroke();
        ctx.setLineDash([]);

        // Draw trail glow
        ctx.fillStyle = colors[i] + '15';
        ctx.beginPath();
        ctx.arc(x, y, 20, 0, Math.PI * 2);
        ctx.fill();

        // Draw ball
        ctx.fillStyle = colors[i];
        ctx.beginPath();
        ctx.arc(x, y, 12, 0, Math.PI * 2);
        ctx.fill();

        // Ball highlight
        ctx.fillStyle = '#ffffff40';
        ctx.beginPath();
        ctx.arc(x - 3, y - 3, 4, 0, Math.PI * 2);
        ctx.fill();

        // Label
        ctx.fillStyle = colors[i];
        ctx.font = '11px system-ui';
        ctx.textAlign = 'center';
        ctx.fillText(labels[i], x, y - 20);
    }
}

function renderRope() {
    var pos = demos.rope.positions();
    var count = demos.rope.particle_count();

    // Draw rope shadow
    ctx.strokeStyle = '#00000040';
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(pos[0] + 2, pos[1] + 2);
    for (var i = 1; i < count; i++) {
        ctx.lineTo(pos[i * 2] + 2, pos[i * 2 + 1] + 2);
    }
    ctx.stroke();

    // Draw rope segments
    ctx.strokeStyle = '#7fdbca';
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    ctx.moveTo(pos[0], pos[1]);
    for (var i = 1; i < count; i++) {
        ctx.lineTo(pos[i * 2], pos[i * 2 + 1]);
    }
    ctx.stroke();

    // Draw particles
    for (var i = 0; i < count; i++) {
        var isPin = (i === 0);
        ctx.fillStyle = isPin ? '#ff6b6b' : '#7fdbca';
        ctx.beginPath();
        ctx.arc(pos[i * 2], pos[i * 2 + 1], isPin ? 7 : 3, 0, Math.PI * 2);
        ctx.fill();

        if (isPin) {
            ctx.strokeStyle = '#ff6b6b60';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(pos[i * 2], pos[i * 2 + 1], 11, 0, Math.PI * 2);
            ctx.stroke();
        }
    }
}

function renderCloth() {
    var cols = demos.cloth.cols();
    var rows = demos.cloth.rows();
    var pos = demos.cloth.positions();

    // Fill triangles for a fabric look
    for (var row = 0; row < rows - 1; row++) {
        for (var col = 0; col < cols - 1; col++) {
            var i0 = (row * cols + col) * 2;
            var i1 = (row * cols + col + 1) * 2;
            var i2 = ((row + 1) * cols + col) * 2;
            var i3 = ((row + 1) * cols + col + 1) * 2;

            // Upper triangle
            ctx.fillStyle = '#7fdbca10';
            ctx.beginPath();
            ctx.moveTo(pos[i0], pos[i0 + 1]);
            ctx.lineTo(pos[i1], pos[i1 + 1]);
            ctx.lineTo(pos[i2], pos[i2 + 1]);
            ctx.closePath();
            ctx.fill();

            // Lower triangle
            ctx.fillStyle = '#7fdbca08';
            ctx.beginPath();
            ctx.moveTo(pos[i1], pos[i1 + 1]);
            ctx.lineTo(pos[i3], pos[i3 + 1]);
            ctx.lineTo(pos[i2], pos[i2 + 1]);
            ctx.closePath();
            ctx.fill();
        }
    }

    // Horizontal lines
    ctx.strokeStyle = '#7fdbca50';
    ctx.lineWidth = 1;
    for (var row = 0; row < rows; row++) {
        ctx.beginPath();
        for (var col = 0; col < cols; col++) {
            var idx = (row * cols + col) * 2;
            if (col === 0) ctx.moveTo(pos[idx], pos[idx + 1]);
            else ctx.lineTo(pos[idx], pos[idx + 1]);
        }
        ctx.stroke();
    }

    // Vertical lines
    for (var col = 0; col < cols; col++) {
        ctx.beginPath();
        for (var row = 0; row < rows; row++) {
            var idx = (row * cols + col) * 2;
            if (row === 0) ctx.moveTo(pos[idx], pos[idx + 1]);
            else ctx.lineTo(pos[idx], pos[idx + 1]);
        }
        ctx.stroke();
    }

    // Draw pinned top row
    for (var col = 0; col < cols; col++) {
        var idx = col * 2;
        ctx.fillStyle = '#ff6b6b';
        ctx.beginPath();
        ctx.arc(pos[idx], pos[idx + 1], 3, 0, Math.PI * 2);
        ctx.fill();
    }
}

function renderSoftBody() {
    var colors = ['#7fdbca', '#ff6b6b', '#ffd93d'];
    var count = demos.softbody.body_count();

    // Draw floor line
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(10, canvas.height - 20);
    ctx.lineTo(canvas.width - 10, canvas.height - 20);
    ctx.stroke();
    ctx.setLineDash([]);

    for (var b = 0; b < count; b++) {
        var pos = demos.softbody.body_positions(b);
        var n = demos.softbody.body_particle_count(b);
        var color = colors[b % colors.length];

        // Fill
        ctx.fillStyle = color + '20';
        ctx.beginPath();
        ctx.moveTo(pos[0], pos[1]);
        for (var i = 1; i < n; i++) {
            ctx.lineTo(pos[i * 2], pos[i * 2 + 1]);
        }
        ctx.closePath();
        ctx.fill();

        // Inner glow
        ctx.fillStyle = color + '10';
        ctx.beginPath();
        ctx.moveTo(pos[0], pos[1]);
        for (var i = 1; i < n; i++) {
            ctx.lineTo(pos[i * 2], pos[i * 2 + 1]);
        }
        ctx.closePath();
        ctx.fill();

        // Outline
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        ctx.moveTo(pos[0], pos[1]);
        for (var i = 1; i < n; i++) {
            ctx.lineTo(pos[i * 2], pos[i * 2 + 1]);
        }
        ctx.closePath();
        ctx.stroke();

        // Particles
        for (var i = 0; i < n; i++) {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(pos[i * 2], pos[i * 2 + 1], 2.5, 0, Math.PI * 2);
            ctx.fill();
        }
    }
}

function updateControls() {
    var controlsEl = document.getElementById('controls');

    if (currentTab === 'cloth') {
        controlsEl.innerHTML =
            '<div class="control-group">' +
            '<label>Wind</label>' +
            '<input type="range" id="wind-slider" min="-200" max="200" value="0" step="10">' +
            '<span class="value" id="wind-value">0</span>' +
            '</div>' +
            '<button class="btn" id="btn-reset-cloth">Reset</button>';

        document.getElementById('wind-slider').addEventListener('input', function(e) {
            windStrength = parseFloat(e.target.value);
            document.getElementById('wind-value').textContent = windStrength;
        });

        document.getElementById('btn-reset-cloth').addEventListener('click', function() {
            demos.cloth = new ClothDemo(15, 12, 25.0);
            windStrength = 0;
            document.getElementById('wind-slider').value = 0;
            document.getElementById('wind-value').textContent = '0';
        });
    } else if (currentTab === 'softbody') {
        controlsEl.innerHTML =
            '<button class="btn" id="btn-reset-soft">Reset</button>';

        document.getElementById('btn-reset-soft').addEventListener('click', function() {
            demos.softbody = new SoftBodyDemo();
        });
    } else if (currentTab === 'rope') {
        controlsEl.innerHTML =
            '<button class="btn" id="btn-reset-rope">Reset</button>';

        document.getElementById('btn-reset-rope').addEventListener('click', function() {
            demos.rope = new RopeDemo(20);
        });
    } else {
        controlsEl.innerHTML = '';
    }
}

function updateInfo() {
    var info = document.getElementById('info');
    var msgs = {
        springs: 'Move mouse to set spring target. Three damping modes follow the cursor.',
        rope: 'Move mouse to drag the pinned end. The rope swings under gravity.',
        cloth: 'Click to tear the cloth. Use the wind slider to apply horizontal force.',
        softbody: 'Click to poke the soft bodies. They bounce under gravity.'
    };
    info.textContent = msgs[currentTab] || '';
}

main().catch(function(err) {
    console.error('Failed to initialize softy demo:', err);
    var info = document.getElementById('info');
    if (info) {
        info.textContent = 'Init error: ' + err.message;
        info.style.color = '#ff6b6b';
    }
});
