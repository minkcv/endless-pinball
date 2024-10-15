var TH = {
    materials : {
        pinkLineMat : null,
        whiteLineMat : null,
        blackBasicMat : null,
        debugBasicMat : null,
        none : null
    },
    threediv : null,
    width : null,
    height : null,
    scene : null,
    camera : null,
    raycaster: null,
    mouseVec : null,
    renderer : null,
    floorY : -30,
    clock : null,
    animators : [],
    lines: [],
    levelTriggers: [],
    fadeOut : false,
    fadeOutDone : null,
    fadeIn : false,
    fadeSpeed : 0.02,
    delta : 0,
    originalRGB: {r: 0, g: 0, b: 0},
    screens: [],

    init : function () {
        TH.threediv = document.getElementById('game'),
        TH.width = TH.threediv.clientWidth;
        TH.height = TH.threediv.clientHeight;
        TH.scene = new THREE.Scene();
        //TH.scene.background = new THREE.Color(0x0c1013);
        TH.camera = new THREE.PerspectiveCamera(45, TH.width / TH.height, 0.1, 8000);
        TH.scene.add(TH.camera);
        TH.camera.rotateY(-3.14 / 2);

        TH.renderer = new THREE.WebGLRenderer({antialias: true});
        TH.renderer.setSize(TH.width, TH.height);
        TH.threediv.appendChild(TH.renderer.domElement);

        TH.raycaster = new THREE.Raycaster();
        TH.mouseVec = new THREE.Vector2();

        TH.clock = new THREE.Clock();

        this.materials.pinkLineMat = new THREE.LineBasicMaterial({color: 0xf442d4});
        this.originalRGB.r = this.materials.pinkLineMat.color.r;
        this.originalRGB.g = this.materials.pinkLineMat.color.g;
        this.originalRGB.b = this.materials.pinkLineMat.color.b;
        // Start black to fade in.
        this.materials.pinkLineMat.color.r = 0;
        this.materials.pinkLineMat.color.g = 0;
        this.materials.pinkLineMat.color.b = 0;
        this.materials.whiteLineMat = new THREE.LineBasicMaterial({color: 0xf260d8});
        this.materials.blackBasicMat = new THREE.MeshBasicMaterial({color: 0x000000, side: THREE.DoubleSide});
        this.materials.debugBasicMat = new THREE.MeshBasicMaterial({color: 0x0c1013, side: THREE.DoubleSide});
        this.materials.none = new THREE.MeshBasicMaterial({color: 0x000000, transparent: true, opacity: 0});
    },
    run : function(update) {
        requestAnimationFrame( function(){TH.run(update)} );
        this.delta = this.clock.getDelta();
        update(this.delta);
        TH.renderer.render( TH.scene, TH.camera );
    },
    resize : function(width, height) {
        TH.renderer.setSize(width, height);
        TH.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 4000);
    },
}

