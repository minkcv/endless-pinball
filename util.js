function distance(p1, p2) {
    var dx = Math.abs(p1.x - p2.x);
    var dy = Math.abs(p1.y - p2.y);
    return Math.sqrt(dx * dx + dy * dy);
}

function updateDebug() {
    document.querySelector('.three > #x-pos').innerHTML = 'X: ' + TH.camera.position.x.toFixed(2);
    document.querySelector('.three > #z-pos').innerHTML = 'Z: ' + TH.camera.position.z.toFixed(2);
    document.querySelector('.three > #angle').innerHTML = 'R: ' + TH.camera.rotation.y.toFixed(2);
    var pos = PL.ball.getPosition();
    var lv = PL.ball.getLinearVelocity();
    document.querySelector('.box2d > #x-pos').innerHTML = 'BALL X: ' + pos.x.toFixed(2);
    document.querySelector('.box2d > #y-pos').innerHTML = 'BALL Y: ' + pos.y.toFixed(2);
    document.querySelector('.box2d > #table-x-pos').innerHTML = 'TABL X: ' + PL.currentTable.x.toFixed(2);
    document.querySelector('.box2d > #table-y-pos').innerHTML = 'TABL Y: ' + PL.currentTable.y.toFixed(2);
    document.querySelector('.box2d > #ball-lv').innerHTML = 'BALL V: ' + lv.lengthSquared().toFixed(2);
}

function getRandomInt(max) {
    return Math.floor(Math.random() * max);
  }