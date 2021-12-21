/////////////////
/// CONSTANTS ///
/////////////////

var canvasSize = [0.49, 0.7]; //Size of each canvas, as a fraction of the total page width and height, respectively
var worldScale = 20; //The width of the entire browser viewport (in meters), determining the scale of the displays

////////////////////////
/// GLOBAL VARIABLES ///
////////////////////////

var worldCanvas; //HTML element object of the world canvas
var mapCanvas; //HTML element object of the map canvas
var worldCtx; //Canvas drawing context of the world canvas
var mapCtx; //Canvas drawing context of the map context

///////////////
/// CLASSES ///
///////////////

//

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
}

function startButtonClick() {
	//
}

function pauseButtonClick() {
	//
}

function resetButtonClick() {
	//
}

function newWorldButtonClick() {
	//
}

/////////////////////
/// EXECUTED CODE ///
/////////////////////

requestAnimationFrame(setup);