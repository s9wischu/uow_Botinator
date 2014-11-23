var CONTINUOUS_SLIDER_RADIUS = 25;
var CONTINUOUS_SLIDER_HANDLE_RADIUS = 8;
var CONTINUOUS_SLIDER_POSITION_OFFSET = CONTINUOUS_SLIDER_RADIUS - CONTINUOUS_SLIDER_HANDLE_RADIUS;
var CONTINUOUS_SLIDER_THETA_OFFSET = Math.PI / 2;
var HEAD_PAN_MAX = 168;
var HEAD_PAN_MIN = -168;
var HEAD_TILT_MAX = 60;
var HEAD_TILT_MIN = -30;

function base_command(x, y, z) {
    var params = {type:"base",x:x, y:y, z:z};
    //$.get("/", params);
}

base_interval = null
function base_command_start(x, y, z) {

    if(base_interval != null) {
        clearInterval(base_interval);
        base_interval = null;
    }

    //append_to_log("Move command start: x=" + x + " y=" + y + " z=" + z + ".");
    var params = {type:"base",x:x, y:y, z:z};
    base_interval = setInterval(function() { $.get("/", params); }, 50);
}
function base_command_end() {
    //append_to_log("Move command end.");
    clearInterval(base_interval);
    base_interval = null;
}

function append_to_log(message) {
    var div = document.getElementById('log');
    div.innerHTML = div.innerHTML + message + "<br>";
}


