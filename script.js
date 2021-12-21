/////////////////
/// CONSTANTS ///
/////////////////

var canvasSize = [0.49, 0.7]; //Size of each canvas, as a fraction of the total page width and height, respectively
var worldScale = 20; //The width of the entire browser viewport (in meters), determining the scale of the displays
var canvasLineWidth = 0.025;
var robotRadius = 0.25;
var robotMarkerTriangleAngle = 30 * (Math.PI / 180); //The front angle of the triangular robot marker
var robotStrokeStyle = "black";
var obstacleStrokeStyle = "black";
var lidarStrokeStyle = "red";
var obstacleSizeRange = [0.5, 3];

////////////////////////
/// GLOBAL VARIABLES ///
////////////////////////

var worldCanvas; //HTML element object of the world canvas
var mapCanvas; //HTML element object of the map canvas
var worldCtx; //Canvas drawing context of the world canvas
var mapCtx; //Canvas drawing context of the map context
var keyStates = {}; //Status of each (keyboard) key

var hasStarted = false; //Used for the control of the tick loop.
var running = false; //Used for the control of the tick loop.
var stop = false; //Used for the control of the tick loop.

var pixelsPerMeter; //Pixels per meter
var worldWidth; //World width in meters
var worldHeight; //World height in meters
var worldMaxX;
var worldMaxY;

///////////////
/// CLASSES ///
///////////////

function Pose(pos, orien) {
	this.pos = pos.slice();
	this.orien = orien; //Given in raidans
}
function Obstacle(pos, orien, width) {
	this.pos = pos.slice();
	this.orien = orien; //Given in raidans
	this.width = width;

	this.draw = function(ctx) {
		var dx = 0.5 * this.width * Math.cos(this.orien);
		var dy = 0.5 * this.width * Math.sin(this.orien);

		console.log(dx);
		console.log(dy);

		ctx.strokeStyle = obstacleStrokeStyle;
		ctx.beginPath();
		ctx.moveTo(this.pos[0] + dx, this.pos[1] + dy); //Top-right
		ctx.lineTo(this.pos[0] + dy, this.pos[1] - dx); //Bottom-right
		ctx.lineTo(this.pos[0] - dx, this.pos[1] - dy); //Bottom-left
		ctx.lineTo(this.pos[0] - dy, this.pos[1] + dx); //Top-left
		ctx.closePath();
		ctx.stroke();
	}
}

/////////////////
/// FUNCTIONS ///
/////////////////

function setup() {
	worldCanvas = document.getElementById("worldCanvas");
	mapCanvas = document.getElementById("mapCanvas");

	worldCanvas.setAttribute("width", String(window.innerWidth * canvasSize[0]) + "px");
	worldCanvas.setAttribute("height", String(window.innerHeight * canvasSize[1]) + "px");
	mapCanvas.setAttribute("width", String(window.innerWidth * canvasSize[0]) + "px");
	mapCanvas.setAttribute("height", String(window.innerHeight * canvasSize[1]) + "px");

	// Create event listeners for the various controls on the page
	document.getElementById("startButton").addEventListener("click", startButtonClick);
	document.getElementById("pauseButton").addEventListener("click", pauseButtonClick);
	document.getElementById("resetButton").addEventListener("click", resetButtonClick);
	document.getElementById("newWorldButton").addEventListener("click", newWorldButtonClick);
	document.addEventListener("keydown", function(e) {
		keydownHandler(e);
	});
	document.addEventListener("keyup", function(e) {
		keyupHandler(e);
	});

	pixelsPerMeter = window.innerWidth / worldScale;
	worldWidth = canvasSize[0] * window.innerWidth / pixelsPerMeter;
	worldHeight = canvasSize[1] * window.innerHeight / pixelsPerMeter;
	worldMaxX = worldWidth / 2;
	worldMaxY = worldHeight / 2;

	// Create the canvas contexts
	worldCtx = worldCanvas.getContext("2d");
	mapCtx = worldCanvas.getContext("2d");

	resetCtx(worldCtx);
	resetCtx(mapCtx);

	generateWorld();
	reset();
}

function startButtonClick() {
	//This is the callback function if you click the start button.
	if(!running && !hasStarted) {
		//If we aren't already running, and we haven't started yet, start for the first time.
		reset();
		running = true;
		hasStarted = true;
		tick(); //This is the actual loop function. You only need to call it once -- it will keep calling itself as appropriate.
	}
	else if(!running && hasStarted) {
		//If we aren't running, but we have started yet, resume where we left off.
		running = true;
		tick(); //This is the actual loop function. You only need to call it once -- it will keep calling itself as appropriate.
	}
	//If we are running, do nothing.
}
function pauseButtonClick() {
	//This is the callback function if you click the pause button.
	if(running) {
		//If you click the pause button while we're running, stop. Otherwise, do nothing.
		stop = true;
	}
	//Note that the actual process of stopping is handled within the tick() function.
	//This just lets the function know to stop the next time around.
}
function resetButtonClick() {
	//This is the callback function if you click the reset button.
	if(!running) {
		hasStarted = false;
		reset(); //Get everything back to its initial state.
	}
}
function newWorldButtonClick() {
	//This is the callback function if you click the new world button.
	if(!running) {
		//If we aren't currently running, generate a new world, and reset.
		//resetContext() isn't needed, because the canvas itself isn't changing.
		clearWorld();
		generateWorld();
		reset();
	}
}
function keydownHandler(e) {
	var keyId = e.which;
	keyStates[keyId] = true;
}
function keyupHandler(e) {
	var keyId = e.which;
	keyStates[keyId] = false;
}

function resetCtx(ctx) {
	ctx.setTransform(1, 0, 0, 1, 0, 0); //Reset the transformation
	ctx.transform(1, 0, 0, -1, 0, 0); //Make y+ point up
	ctx.transform(1, 0, 0, 1, ctx.canvas.width/2, -ctx.canvas.height/2); //Center 0,0 in the middle of the canvas
	ctx.transform(pixelsPerMeter, 0, 0, pixelsPerMeter, 0, 0); //Scale according browser scaling
	ctx.lineWidth = canvasLineWidth; //Set the appropriate line width
}
function clearCanvas(ctx) {
	var tf = ctx.getTransform(); //Get the current transformation
	ctx.setTransform(1, 0, 0, 1, 0, 0); //Reset the transformation
	ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height); //Clear the canvas
	ctx.setTransform(tf); //Restore the previous transformation
}
function drawRobot(ctx, pose) {
	//pose should be a Pose object
	//Draw the outer circle
	ctx.strokeStyle = robotStrokeStyle;
	ctx.beginPath();
	//We initially move to a position on the circle itself, so that there's no weird line from the center to the circle.
	//Just JavaScript things! :)
	ctx.moveTo(pose.pos[0] + robotRadius, pose.pos[1]);
	ctx.arc(pose.pos[0], pose.pos[1], robotRadius, 0, 2*Math.PI, true);
	ctx.stroke();

	//Draw a triangle showing orientation.
	//First, compute the coordinates of the three points.
	var dx = robotRadius * Math.cos(pose.orien)
	var dy = robotRadius * Math.sin(pose.orien)
	var front = [pose.pos[0] + dx, pose.pos[1] + dy]

	var backLeftAngle = pose.orien + Math.PI - robotMarkerTriangleAngle;
	dx = robotRadius * Math.cos(backLeftAngle);
	dy = robotRadius * Math.sin(backLeftAngle);
	var backLeft = [pose.pos[0] + dx, pose.pos[1] + dy];

	var backRightAngle = pose.orien + Math.PI + robotMarkerTriangleAngle;
	dx = robotRadius * Math.cos(backRightAngle);
	dy = robotRadius * Math.sin(backRightAngle);
	var backRight = [pose.pos[0] + dx, pose.pos[1] + dy];
	
	//Now actually draw the triangle.
	ctx.beginPath();
	ctx.moveTo(front[0], front[1]);
	ctx.lineTo(backLeft[0], backLeft[1]);
	ctx.lineTo(backRight[0], backRight[1]);
	ctx.lineTo(front[0], front[1]);
	ctx.stroke();
}

function reset() {
	//
}
function tick() {
	//This is the function where it all happens. It will repeatedly call itself until stopped.
	if(stop) {
		//Stop is set to true if the user pauses.
		running = false;
		stop = false;
		return; //We return early, so all the code isn't executed.
	}

	requestAnimationFrame(tick);
}

function clearWorld() {
	//
}
function generateWorld() {
	//
}

/////////////////////
/// EXECUTED CODE ///
/////////////////////

requestAnimationFrame(setup);