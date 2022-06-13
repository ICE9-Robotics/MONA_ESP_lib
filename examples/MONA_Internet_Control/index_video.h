const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
    <head>
        <meta charset="UTF-8">
        
        <script type="text/javascript">

document.addEventListener('keydown', function(e) {
    switch (e.keyCode) {
        case 37:
            sendData("L")
            break;
        case 38:
           sendData("U")
            break;
        case 39:
            sendData("R")
            break;
        case 40:
            sendData("D")
            break;
	default:
	    sendData("STOP")
    }
});
         
            function sendData(motorData) {
                var xhttp = new XMLHttpRequest();
                xhttp.onreadystatechange = function() {
                    if (this.readyState == 4 && this.status == 200) {
                    }
                };
                xhttp.open("GET", "setMotors?motorState="+motorData, true);
                xhttp.send();
            }
            
          </script>
    </head>
    <body>
        <h2>MONA Remote Control</h2>
         <iframe src="***********" width="550" height="380" autoplay> Sorry your browser does not support embedding videos. </iframe> <!-- Link to camera stream. Take care of port forwarding and where the html is accessed from -->
         <br/>
        <ul> 
            <li>Press an arrow key to start moving in that direction.</li>
            <li>Press any non-arrow key to stop moving.</li>
        </ul>
   </body>
</html>

)=====";
