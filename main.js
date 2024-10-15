function update(delta) {
    stats.begin();
    if (debug) {
        updateDebug();
    }
    PL.update(delta);
    stats.end();
}

function init() {
    PL.init();
    TH.init();
}
init();
TH.run(update);
