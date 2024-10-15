var PL = {
    world : null,
    testbed : null,
    ball : null,
    ballFixture : null,
    ramps : [],
    leftFlipper : null,
    leftMotor : null,
    rightFlipper : null,
    rightMotor : null,
    midLeftFlipper : null,
    midRightFlipper : null,
    midLeftMotor : null,
    midRightMotor : null,
    scale : 0.1,
    currentTable : null,
    topTable : null,
    leftTable : null,
    rightTable : null,
    bottomTable : null,
    flipperIndex : {left: 0, right: 1, midLeft: 2, midRight: 3},
    tableEdge : {left: 0, right: 1, top: 2, bottom: 3},
    ballStates : {outLeft: 0, outRight: 1, outTop: 2, outBottom: 3, inPlay: 4},
    ballState : 4,
    tableWidth : 46,
    tableHeight : 84.2,
    slingStrength: 4,
    popStrength: 10,
    kickStrength: 10,
    init : function() {
        const Vec2 = planck.Vec2;
        PL.world = new planck.World({gravity: new Vec2(0.0, -9.8)});
        PL.testbed = planck.Testbed.mount();
        PL.world.on('begin-contact', function(contact) {
            var manifold = contact.getManifold();
            var fixtureA = contact.getFixtureA();
            var fixtureB = contact.getFixtureB();
            PL.ramps.forEach((r) => {
                if ((fixtureA == PL.ballFixture && fixtureB == r) ||
                (fixtureA == r && fixtureB == PL.ballFixture)) {
                    PL.ballFixture.setFilterCategoryBits(r.getUserData().to);
                }
            });
            PL.currentTable.loaders.forEach((l) => {
                if ((fixtureA == PL.ballFixture && fixtureB == l ) ||
                    (fixtureA == l && fixtureB == PL.ballFixture)) {
                    var edge = l.getUserData().edge;
                    if (edge == PL.tableEdge.left && PL.leftTable == null) {
                        PL.leftTable = PL.newTable(PL.currentTable.x - PL.tableWidth, PL.currentTable.y);
                        PL.leftTable.needsCreation = true;
                    }
                    else if (edge == PL.tableEdge.right && PL.rightTable == null) {
                        PL.rightTable = PL.newTable(PL.currentTable.x + PL.tableWidth, PL.currentTable.y);
                        PL.rightTable.needsCreation = true;
                    }
                    else if (edge == PL.tableEdge.top && PL.topTable == null) {
                        PL.topTable = PL.newTable(PL.currentTable.x, PL.currentTable.y + PL.tableHeight);
                        PL.topTable.needsCreation = true;
                    }
                    else if (edge == PL.tableEdge.bottom && PL.bottomTable == null) {
                        PL.bottomTable = PL.newTable(PL.currentTable.x, PL.currentTable.y - PL.tableHeight);
                        PL.bottomTable.needsCreation = true;
                    }
                }
            });
            // Check sensors first
            if (manifold.pointCount == 0) {
                return;
            }
            PL.currentTable.bodies.forEach((b) => {
                var f = b.getFixtureList();
                while(f) {
                    if ((fixtureA == PL.ballFixture && fixtureB == f ) ||
                    (fixtureA == f && fixtureB == PL.ballFixture)) {
                        if (f.getUserData() != null && f.getUserData().strength !== undefined) {
                            // bumper
                            var vl = PL.ball.getLinearVelocity().lengthSquared();
                            if (f.getUserData().sling && vl < 40){
                                f = f.getNext();
                                continue; // slings don't activate if ball is slow
                            }
                            var force = new Vec2(PL.ball.getWorldCenter()).sub(new Vec2(f.getUserData().x + PL.currentTable.x, f.getUserData().y + PL.currentTable.y));
                            force.normalize();
                            force.mul(f.getUserData().strength);
                            if (Vec2.lengthSquared(force) > 0) {
                                PL.ball.applyLinearImpulse(force, PL.ball.getWorldCenter());
                            }
                        }
                    }
                    f = f.getNext();
                }
            });
        });
        PL.world.on('end-contact', function(contact) {
            var manifold = contact.getManifold();
            var fixtureA = contact.getFixtureA();
            var fixtureB = contact.getFixtureB();
            PL.currentTable.edges.forEach((e) => {
                if ((fixtureA == PL.ballFixture && fixtureB == e ) ||
                (fixtureA == e && fixtureB == PL.ballFixture)) {
                    var edge = e.getUserData().edge;
                    // attempt 1: make sure ball is leaving on the outside of the sensor,
                    // note: the ball leaves the edge trigger on the next table as it enters
                    //       ideally the current table has been changed at that point
                    if (edge == PL.tableEdge.left && PL.ball.getWorldCenter().x < e.getBody().getWorldCenter().x) {
                        PL.ballState = edge;
                    }
                    else if (edge == PL.tableEdge.right && PL.ball.getWorldCenter().x > e.getBody().getWorldCenter().x) {
                        PL.ballState = edge;
                    }
                    else if (edge == PL.tableEdge.top && PL.ball.getWorldCenter().y > e.getBody().getWorldCenter().y) {
                        PL.ballState = edge;
                    }
                    else if (edge == PL.tableEdge.bottom && PL.ball.getWorldCenter().y < e.getBody().getWorldCenter().y) {
                        PL.ballState = edge;
                    }
                }
            });
            PL.currentTable.loaders.forEach((l) => {
                if ((fixtureA == PL.ballFixture && fixtureB == l ) ||
                    (fixtureA == l && fixtureB == PL.ballFixture)) {
                    var edge = l.getUserData().edge;
                    if (edge == PL.tableEdge.left && PL.ball.getWorldCenter().x > PL.currentTable.x) {
                        PL.leftTable.needsDestruction = true;
                    }
                    else if (edge == PL.tableEdge.right && PL.ball.getWorldCenter().x < PL.currentTable.x + PL.tableWidth) {
                        PL.rightTable.needsDestruction = true;
                    }
                    else if (edge == PL.tableEdge.top && PL.ball.getWorldCenter().y < PL.currentTable.y + PL.tableHeight) {
                        PL.topTable.needsDestruction = true;
                    }
                    else if (edge == PL.tableEdge.bottom && PL.ball.getWorldCenter().y > PL.currentTable.y) {
                        PL.bottomTable.needsDestruction = true;
                    }
                }
            });
        });
        // test bounds
        /*
        const Edge = planck.Edge;
        var w = 46;
        var h = 84.2;
        var platform = PL.world.createBody({
            type: "static",
            position: new Vec2(0.0, 0.0)
        });
        platform.createFixture({
            shape: new Edge(new Vec2(0, 0), new Vec2(w, 0)),
        });
        platform.createFixture({
            shape: new Edge(new Vec2(w, 0), new Vec2(w, h)),
        });
        platform.createFixture({
            shape: new Edge(new Vec2(0, 0), new Vec2(0, h)),
        });
        platform.createFixture({
            shape: new Edge(new Vec2(0, h), new Vec2(w, h)),
        });
        */
        PL.addBall(11, 9);
        PL.currentTable = PL.newTable(0, 0);
        PL.createTable(PL.currentTable);
        PL.assignFlippers();
        
        //PL.testbed.width = 60;
        //PL.testbed.height = 50;
        PL.testbed.width = 46;
        PL.testbed.height = 84;
        PL.testbed.x = 23;
        PL.testbed.y = -42;
        PL.testbed.start(PL.world);
    },
    update : function(timeStep) {
        var velocityIterations = 10; // 10
        var positionIterations = 8; // 8
        PL.world.step(timeStep, velocityIterations, positionIterations);
        var flipLeft = keys.lctrl in keysDown || keys.a in keysDown || keys.left in keysDown;
        var motorSpeed = 480;
        if (flipLeft) {
            PL.leftMotor.setMotorSpeed(motorSpeed);
            if (PL.midLeftMotor != null)
                PL.midLeftMotor.setMotorSpeed(motorSpeed);
        } else {
            PL.leftMotor.setMotorSpeed(-motorSpeed);
            if (PL.midLeftMotor != null)
                PL.midLeftMotor.setMotorSpeed(-motorSpeed);
        }
        var flipRight = keys.rctrl in keysDown || keys.d in keysDown || keys.right in keysDown;
        if (flipRight) {
            PL.rightMotor.setMotorSpeed(-motorSpeed);
            if (PL.midRightMotor != null)
                PL.midRightMotor.setMotorSpeed(-motorSpeed);
        } else {
            PL.rightMotor.setMotorSpeed(motorSpeed);
            if (PL.midRightMotor != null)
                PL.midRightMotor.setMotorSpeed(motorSpeed);
        }
        if (PL.ballState != PL.ballStates.inPlay) {
            if (PL.ballState == PL.ballStates.outLeft) {
                PL.rightTable = PL.currentTable;
                PL.currentTable = PL.leftTable;
                PL.leftTable = null;
            }
            else if (PL.ballState == PL.ballStates.outRight) {
                PL.leftTable = PL.currentTable;
                PL.currentTable = PL.rightTable;
                PL.rightTable = null;
            }
            else if (PL.ballState == PL.ballStates.outTop) {
                PL.bottomTable = PL.currentTable;
                PL.currentTable = PL.topTable;
                PL.topTable = null;
            }
            else if (PL.ballState == PL.ballStates.outBottom) {
                PL.topTable = PL.currentTable;
                PL.currentTable = PL.bottomTable;
                PL.bottomTable = null;
            }
                
            PL.ballState = PL.ballStates.inPlay;
            PL.assignFlippers();
        }
        else {
            // in play
            if (PL.leftTable && PL.leftTable.needsCreation)
                PL.createTable(PL.leftTable);
            if (PL.leftTable && PL.leftTable.needsDestruction) {
                PL.destroyTable(PL.leftTable);
                PL.leftTable = null;
            }
            if (PL.rightTable && PL.rightTable.needsCreation)
                PL.createTable(PL.rightTable);
            if (PL.rightTable && PL.rightTable.needsDestruction) {
                PL.destroyTable(PL.rightTable);
                PL.rightTable = null;
            }
            if (PL.topTable && PL.topTable.needsCreation)
                PL.createTable(PL.topTable);
            if (PL.topTable && PL.topTable.needsDestruction) {
                PL.destroyTable(PL.topTable);
                PL.topTable = null;
            }
            if (PL.bottomTable && PL.bottomTable.needsCreation)
                PL.createTable(PL.bottomTable);
            if (PL.bottomTable && PL.bottomTable.needsDestruction) {
                PL.destroyTable(PL.bottomTable);
                PL.bottomTable = null;
            }
        }
        // camera follows ball
        //var pos = PL.ball.getPosition();
        //PL.testbed.x = pos.x;
        //PL.testbed.y = -pos.y;
        // camera centers on current table
        PL.testbed.x = PL.currentTable.x + 23;
        PL.testbed.y = -PL.currentTable.y - 42;
    },
    assignFlippers : function() {
        PL.currentTable.joints.forEach((j) => {
            var index = j.getUserData().flipperIndex;
            if (index == PL.flipperIndex.left) {
                PL.leftMotor = j;
            }
            else if (index == PL.flipperIndex.right){
                PL.rightMotor = j;
            }
            else if (index == PL.flipperIndex.midLeft) {
                PL.midLeftMotor  = j;
            }
            else {
                PL.midRightMotor = j;
            }
        });
    },
    addBall : function(x, y) {
        PL.ball = PL.world.createBody({
            type: "dynamic",
            position: new planck.Vec2(x, y),
            bullet: true,
        });
        var shape = new planck.CircleShape(
            new planck.Vec2(0, 0),
            1.3
        );
        PL.ballFixture = PL.ball.createFixture({
            shape: shape,
            friction: 0.15,
            restitution: 0,
            density: 0.025,
            filterCategoryBits: 0x0001,
            filterMaskBits: 0xFFFF
        });
    },
    newTable : function(x, y) {
        var table = {
            bodies: [],
            joints: [],
            edges: [],
            flippers: [],
            loaders: [],
            x: x,
            y: y
        };
        return table;
    },
    createTable : function(table) {
        var x = table.x;
        var y = table.y;
        PL.addReturns(table, x, y);
        var middleLeft = getRandomInt(3);
        if (middleLeft == 0)
            PL.addMiddleLeft1(table, x, y);
        else if (middleLeft == 1)
            PL.addMiddleLeft2(table, x, y);
        else
            PL.addMiddleLeft3(table, x, y);
        var middleRight = getRandomInt(3);
        if (middleRight == 0)
            PL.addMiddleRight1(table, x, y);
        else if (middleRight == 1)
            PL.addMiddleRight2(table, x, y);
        else
            PL.addMiddleRight3(table, x, y);
        var outfield = getRandomInt(3);
        if (outfield == 0)
            PL.addOutfield1(table, x, y);
        else if (outfield == 1)
            PL.addOutfield2(table, x, y);
        else
            PL.addOutfield3(table, x, y);

        var leftEdge = PL.addWall(table, {x: 0, y: 41.9}, {x: 0, y: 45.7}, 1, {edge: PL.tableEdge.left});
        var rightEdge = PL.addWall(table, {x: 46, y: 41.9}, {x: 46, y: 45.7}, 1, {edge: PL.tableEdge.right});
        var bottomEdge = PL.addWall(table, {x: 12.4, y: 0}, {x: 33.6, y: 0}, 1, {edge: PL.tableEdge.bottom});
        var topEdge = PL.addWall(table, {x: 12.4, y: 84.2}, {x: 33.6, y: 84.2}, 1, {edge: PL.tableEdge.top});
        leftEdge.setSensor(true);
        rightEdge.setSensor(true);
        bottomEdge.setSensor(true);
        topEdge.setSensor(true);
        table.edges.push(leftEdge);
        table.edges.push(rightEdge);
        table.edges.push(topEdge);
        table.edges.push(bottomEdge);

        PL.addLoader(table, PL.tableEdge.left);
        PL.addLoader(table, PL.tableEdge.right);
        PL.addLoader(table, PL.tableEdge.top);
        PL.addLoader(table, PL.tableEdge.bottom);
        table.needsCreation = false;
    },
    destroyTable : function(table) {
        table.bodies.forEach((b) => 
            PL.world.destroyBody(b));
        table.joints.forEach((j) => 
            PL.world.destroyJoint(j));
        table.edges.forEach((e) => 
            PL.world.destroyBody(e.getBody()));
        table.loaders.forEach((l) => 
            PL.world.destroyBody(l.getBody()));
        table.needsDestruction = false;
    },
    addLoader(table, edge) {
        var w, h, x, y;
        w = 3;
        h = 5;
        if (edge == PL.tableEdge.left) {
            x = table.x + (w / 2);
            y = table.y + (PL.tableHeight / 2) + (h / 2);
        }
        else if (edge == PL.tableEdge.right) {
            x = table.x + PL.tableWidth - (w / 2);
            y = table.y + (PL.tableHeight / 2) + (h / 2);
        }
        else if (edge == PL.tableEdge.top) {
            w = 20;
            h = 10;
            x = table.x + (PL.tableWidth / 2);
            y = table.y + PL.tableHeight - (h / 2)
        }
        else if (edge == PL.tableEdge.bottom) {
            w = 40;
            h = 15;
            x = table.x + (PL.tableWidth / 2);
            y = table.y + (h / 2);
        }
        var body = PL.world.createBody(new planck.Vec2(x, y), 0);
        var loader = body.createFixture({
            shape: new planck.Box(w / 2.0, h / 2.0, new planck.Vec2(0, 0)),
            isSensor: true,
            userData: {edge: edge}
        });
        table.loaders.push(loader)
    },
    addFlipper : function(table, x, y, angle, index, layer) {
        if (layer === undefined) 
            layer = 0x0001;
        const Vec2 = planck.Vec2;
        var points = [
            Vec2(-8.4, -1),
            Vec2(-8.8, 0.8), 
            Vec2(-5, 72), 
            Vec2(-3, 76), 
            Vec2(-1.5, 79), 
            Vec2(1.5, 79), 
            Vec2(3, 76), 
            Vec2(5, 72), 
            Vec2(8.8, 0.8),
            Vec2(8.4, -1),
        ];
        points.forEach((p) => {p.x *= PL.scale * 1.35; p.y *= PL.scale * 0.85;});
        var poly = new planck.Polygon(points);
        var flipper = PL.world.createBody({
            position: new Vec2(table.x + x, table.y + y),
            type: "dynamic",
            angle: angle,
            //allowSleep: true
            //awake: false // is this much of a performance improvement?
        });
        var fixture = flipper.createFixture({
            shape: poly, 
            density: 0.3, 
            restitution: 0.4, 
            friction: 7,
            filterCategoryBits: layer,
            filterMaskBits: layer,
            userData: {flipperIndex: index}
        });
        var jd = {userData: {flipperIndex: index}};
        jd.enableMotor = true;
        jd.maxMotorTorque = 20000.0;
        jd.enableLimit = true;
        //jd.motorSpeed = 1.1;
        var circle = PL.world.createBody({position: new Vec2(table.x + x, table.y + y)});
        circle.createFixture({
            shape: new planck.CircleShape(
                new planck.Vec2(0, 0),
                1.2
            ),
            filterCategoryBits: layer,
            filterMaskBits: layer
        });
        jd.lowerAngle = -28.5 * Math.PI / 180.0;
        jd.upperAngle = 28.5 * Math.PI / 180.0;
        var motor = new planck.RevoluteJoint(jd, circle, flipper, flipper.getPosition());
        PL.world.createJoint(motor);

        table.bodies.push(flipper);
        table.bodies.push(circle);
        table.joints.push(motor);
        table.flippers.push(fixture);
    },
    // bumpCenter: world Vec2
    addBumperWall : function(table, p1, p2, layer, bumpCenter) {
        var bumper = PL.addWall(table, p1, p2, layer, bumpCenter);
        //bumper.setRestitution(1);
        table.bodies.push(bumper.getBody());
        return bumper;
    },
    addWall : function(table, p1, p2, layer, data) {
        if (layer === undefined) 
        layer = 0x0001;
        var length = distance(p1, p2);
        var x = (p2.x + p1.x) / 2;
        var y = (p2.y + p1.y) / 2;
        // horizontal until rotated
        var angle = -Math.atan((p1.y - p2.y) / (p2.x - p1.x));
        var body = PL.world.createBody({x: table.x + x, y: table.y + y}, 0);
        table.bodies.push(body);
        return body.createFixture({
            shape: new planck.BoxShape(length / 2, 0.1, new planck.Vec2(0, 0), angle),
            filterCategoryBits: 0xFFFF,
            filterMaskBits: layer,
            userData: data
        });
    },
    addPopBumper : function(table, x, y, layer, radius) {
        if (radius === undefined)
            radius = 2.2;
        var bumper = PL.world.createBody({position: new planck.Vec2(table.x + x, table.y + y)});
        var circle = bumper.createFixture({
            shape: new planck.CircleShape(
                new planck.Vec2(0, 0),
                radius,
            ),
            restitution: 0,//1.5,
            friction: 0,
            filterCategoryBits: 0xFFFF,
            filterMaskBits: layer,
            userData: {x: x, y: y, strength: PL.popStrength}
        });
        table.bodies.push(bumper);
        return bumper;
    },
    addRamp : function(table, p1, p2, from, to) {
        var fixture = PL.addWall(table, p1, p2, from, {from: from, to: to});
        fixture.setSensor(true);
        PL.ramps.push(fixture);
        return fixture.getBody();
    },
    addShape : function(table, x, y, points, layer, friction, restitution) {
        if (layer === undefined) 
            layer = 0x0001;
        if (friction === undefined)
            friction = 0.01;
        if (restitution === undefined)
            restitution = 0.1;
        points.forEach((p) => {p.x *= PL.scale; p.y *= PL.scale;});
        const Vec2 = planck.Vec2;
        var chain = new planck.Chain(points, true);
        var body = PL.world.createBody({
            position: new Vec2(x, y)
        });
        body.createFixture({
            shape: chain,
            friction: friction,
            restitution: restitution,
            filterCategoryBits: 0xFFFF,
            filterMaskBits: layer
        });
        table.bodies.push(body);
        return body;
    },
    addReturns : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // left inlane
        var vs = [Vec2(44.7, 216.4), Vec2(44.0, 215.7), Vec2(44.0, 154.0), Vec2(44.0, 92.4), Vec2(45.3, 90.8), Vec2(46.5, 89.2), Vec2(84.3, 63.6), Vec2(122.1, 37.9), Vec2(123.6, 37.3), Vec2(125.0, 36.8), Vec2(125.0, 38.4), Vec2(125.0, 40.1), Vec2(127.9, 45.9), Vec2(130.9, 51.7), Vec2(134.0, 53.0), Vec2(137.1, 54.3), Vec2(139.7, 53.1), Vec2(142.3, 51.9), Vec2(142.8, 52.5), Vec2(143.4, 53.0), Vec2(141.4, 54.4), Vec2(139.5, 55.8), Vec2(106.5, 76.1), Vec2(73.5, 96.3), Vec2(68.2, 100.9), Vec2(63.0, 105.4), Vec2(58.4, 111.3), Vec2(53.8, 117.1), Vec2(52.2, 121.8), Vec2(50.6, 126.5), Vec2(50.0, 171.5), Vec2(49.5, 216.5), Vec2(47.5, 216.8), Vec2(45.4, 217.1), Vec2(44.7, 216.4)];
        PL.addShape(table, x, y, vs, 1);
        // right inlane
        vs = [Vec2(410.7, 216.4), Vec2(410.0, 215.7), Vec2(410.0, 174.1), Vec2(410.0, 132.5), Vec2(408.5, 125.6), Vec2(407.0, 118.7), Vec2(404.3, 114.6), Vec2(401.5, 110.5), Vec2(394.8, 103.9), Vec2(388.1, 97.4), Vec2(359.8, 80.0), Vec2(331.5, 62.7), Vec2(323.7, 57.7), Vec2(315.9, 52.8), Vec2(316.4, 52.2), Vec2(316.9, 51.7), Vec2(320.0, 53.0), Vec2(323.0, 54.2), Vec2(326.1, 53.0), Vec2(329.1, 51.7), Vec2(332.1, 45.9), Vec2(335.0, 40.1), Vec2(335.0, 38.4), Vec2(335.0, 36.8), Vec2(336.4, 37.3), Vec2(337.9, 37.9), Vec2(375.7, 63.6), Vec2(413.5, 89.2), Vec2(414.8, 90.8), Vec2(416.0, 92.4), Vec2(415.8, 154.5), Vec2(415.5, 216.5), Vec2(413.5, 216.8), Vec2(411.4, 217.1), Vec2(410.7, 216.4)];
        PL.addShape(table, x, y, vs, 1);
        // left bumper
        vs = [Vec2(83.9, 220.1), Vec2(81.0, 217.2), Vec2(81.0, 175.6), Vec2(81.0, 134.0), Vec2(99.8, 119.9), Vec2(118.5, 105.8), Vec2(121.8, 106.4), Vec2(125.2, 106.9), Vec2(126.6, 108.1), Vec2(128.0, 109.2), Vec2(128.0, 112.6), Vec2(128.0, 116.0), Vec2(125.8, 118.2), Vec2(123.7, 120.3), Vec2(109.3, 165.4), Vec2(95.0, 210.5), Vec2(95.0, 214.3), Vec2(95.0, 218.1), Vec2(92.5, 220.5), Vec2(90.1, 223.0), Vec2(88.5, 223.0), Vec2(86.8, 223.0), Vec2(83.9, 220.1)];
        PL.addShape(table, x, y, vs, 1, 5, 0.5);
        // right bumper
        vs = [Vec2(367.5, 220.5), Vec2(365.0, 218.1), Vec2(365.0, 214.3), Vec2(365.0, 210.5), Vec2(350.7, 165.4), Vec2(336.3, 120.3), Vec2(334.2, 118.2), Vec2(332.0, 116.0), Vec2(332.0, 112.6), Vec2(332.0, 109.2), Vec2(333.4, 108.1), Vec2(334.8, 106.9), Vec2(338.2, 106.4), Vec2(341.5, 105.8), Vec2(360.3, 119.9), Vec2(379.0, 134.0), Vec2(379.0, 175.6), Vec2(379.0, 217.2), Vec2(376.1, 220.1), Vec2(373.2, 223.0), Vec2(371.5, 223.0), Vec2(369.9, 223.0), Vec2(367.5, 220.5)];
        PL.addShape(table, x, y, vs, 1, 5, 0.5);
        // left outlane
        vs = [Vec2(0, 120.5), Vec2(0, 0), Vec2(62.2, 0), Vec2(124.4, 0), Vec2(123.0, 1.4), Vec2(121.5, 2.7), Vec2(89.0, 25.5), Vec2(56.5, 48.4), Vec2(53.5, 50.4), Vec2(50.5, 52.5), Vec2(34.3, 63.9), Vec2(18.0, 75.3), Vec2(17.0, 78.9), Vec2(16.0, 82.5), Vec2(16.0, 161.8), Vec2(16.0, 241.0), Vec2(8.0, 241.0), Vec2(0, 241.0), Vec2(0, 120.5)];
        PL.addShape(table, x, y, vs, 1);
        // right outlane
        vs = [Vec2(444.0, 161.8), Vec2(444.0, 82.5), Vec2(443.0, 78.9), Vec2(442.0, 75.3), Vec2(425.7, 63.9), Vec2(409.5, 52.5), Vec2(406.5, 50.4), Vec2(403.5, 48.4), Vec2(371.0, 25.5), Vec2(338.5, 2.7), Vec2(337.0, 1.4), Vec2(335.6, 0), Vec2(397.8, 0), Vec2(460.0, 0), Vec2(460.0, 120.5), Vec2(460.0, 241.0), Vec2(452.0, 241.0), Vec2(444.0, 241.0), Vec2(444.0, 161.8)];
        PL.addShape(table, x, y, vs, 1);

        var stretch = 3;
        // right sling
        PL.addBumperWall(table, {x: 36.5, y: 21.5}, {x: 33.4, y: 11.7}, 1, {x: 37.8 + stretch, y: 15.3, strength: PL.slingStrength, sling: true});
        // left sling
        PL.addBumperWall(table, {x: 9.4, y: 21.5}, {x: 12.5, y: 11.7}, 1, {x: 8.1 - stretch, y: 15.3, strength: PL.slingStrength, sling: true});

        // return rails
        // left side return rail
        vs = [Vec2(0, 386.3), Vec2(0, 384.5), Vec2(30.2, 353.5), Vec2(60.3, 322.5), Vec2(70.6, 311.9), Vec2(81.0, 301.3), Vec2(81.0, 244.2), Vec2(81.0, 187.0), Vec2(65.0, 187.0), Vec2(49.0, 187.0), Vec2(49.0, 237.3), Vec2(49.0, 287.7), Vec2(24.8, 312.5), Vec2(0.5, 337.3), Vec2(0.2, 334.5), Vec2(-0.2, 331.7), Vec2(22.4, 309.1), Vec2(45.0, 286.5), Vec2(45.0, 234.7), Vec2(45.0, 183.0), Vec2(65.0, 183.0), Vec2(85.0, 183.0), Vec2(85.0, 243.1), Vec2(85.0, 303.2), Vec2(43.8, 345.3), Vec2(2.6, 387.5), Vec2(1.3, 387.8), Vec2(0, 388.1), Vec2(0, 386.3)];
        //PL.addShape(table, x, y, vs, 2);
        // right side return rail
        vs = [Vec2(416.3, 345.5), Vec2(375.0, 303.2), Vec2(375.0, 243.1), Vec2(375.0, 183.0), Vec2(395.0, 183.0), Vec2(415.0, 183.0), Vec2(415.0, 234.7), Vec2(415.0, 286.5), Vec2(437.6, 309.1), Vec2(460.2, 331.7), Vec2(459.8, 334.5), Vec2(459.5, 337.3), Vec2(435.3, 312.5), Vec2(411.0, 287.7), Vec2(411.0, 237.3), Vec2(411.0, 187.0), Vec2(395.0, 187.0), Vec2(379.0, 187.0), Vec2(379.0, 244.2), Vec2(379.0, 301.3), Vec2(389.4, 311.9), Vec2(399.7, 322.5), Vec2(429.8, 353.5), Vec2(460.0, 384.5), Vec2(460.0, 386.3), Vec2(460.0, 388.0), Vec2(458.8, 387.9), Vec2(457.5, 387.9), Vec2(416.3, 345.5)];
        //PL.addShape(table, x, y, vs, 2);

        // left straight return rail
        vs = [Vec2(118.5, 452.3), Vec2(117.9, 447.5), Vec2(116.8, 442.0), Vec2(115.8, 436.5), Vec2(112.8, 428.0), Vec2(109.8, 419.5), Vec2(104.8, 409.6), Vec2(99.9, 399.7), Vec2(94.6, 391.6), Vec2(89.4, 383.5), Vec2(81.2, 372.5), Vec2(73.0, 361.5), Vec2(67.6, 350.5), Vec2(62.1, 339.5), Vec2(58.7, 329.0), Vec2(55.2, 318.5), Vec2(53.0, 308.5), Vec2(50.9, 298.5), Vec2(48.8, 285.5), Vec2(46.8, 272.5), Vec2(46.3, 234.7), Vec2(45.8, 197.0), Vec2(46.4, 190.6), Vec2(47.1, 184.3), Vec2(64.8, 183.7), Vec2(82.4, 183.1), Vec2(83.5, 226.6), Vec2(84.0, 269.5), Vec2(85.5, 280.8), Vec2(87.1, 292.1), Vec2(89.5, 301.4), Vec2(91.9, 310.8), Vec2(94.3, 317.1), Vec2(96.7, 323.5), Vec2(100.5, 331.0), Vec2(104.3, 338.5), Vec2(107.3, 343.0), Vec2(110.4, 347.5), Vec2(118.4, 357.5), Vec2(126.4, 367.5), Vec2(130.9, 374.5), Vec2(135.4, 381.5), Vec2(141.1, 393.0), Vec2(146.9, 404.5), Vec2(150.9, 416.5), Vec2(154.8, 428.5), Vec2(156.9, 439.3), Vec2(159.0, 450.1), Vec2(159.0, 453.6), Vec2(159.0, 457.0), Vec2(157.0, 457.0), Vec2(155.0, 457.0), Vec2(155.0, 453.4), Vec2(155.0, 449.8), Vec2(152.9, 439.7), Vec2(150.9, 429.5), Vec2(146.9, 417.5), Vec2(142.9, 405.5), Vec2(136.8, 393.3), Vec2(130.7, 381.1), Vec2(128.9, 379.1), Vec2(127.0, 377.1), Vec2(120.7, 367.3), Vec2(114.5, 359.2), Vec2(108.4, 351.2), Vec2(102.3, 343.3), Vec2(97.3, 333.4), Vec2(92.2, 323.5), Vec2(89.1, 314.1), Vec2(86.0, 304.8), Vec2(83.5, 292.6), Vec2(81.1, 280.5), Vec2(80.0, 266.0), Vec2(79.0, 251.5), Vec2(79.0, 219.3), Vec2(79.0, 187.2), Vec2(65.3, 187.7), Vec2(51.5, 188.3), Vec2(50.8, 189.4), Vec2(50.0, 190.5), Vec2(50.0, 224.0), Vec2(50.0, 257.5), Vec2(51.0, 269.0), Vec2(52.1, 280.5), Vec2(54.0, 292.5), Vec2(56.0, 304.5), Vec2(59.0, 315.9), Vec2(62.0, 327.3), Vec2(66.0, 337.4), Vec2(70.1, 347.5), Vec2(74.6, 355.3), Vec2(79.1, 363.1), Vec2(87.4, 374.3), Vec2(95.7, 385.5), Vec2(99.3, 391.3), Vec2(103.0, 397.0), Vec2(108.1, 407.1), Vec2(113.1, 417.2), Vec2(115.9, 424.9), Vec2(118.8, 432.5), Vec2(120.3, 440.0), Vec2(121.9, 447.5), Vec2(122.5, 452.3), Vec2(123.1, 457.0), Vec2(121.1, 457.0), Vec2(119.1, 457.0), Vec2(118.5, 452.3)];
        PL.addShape(table, x, y, vs, 2);
        // right straight return rail
        vs = [Vec2(301.0, 453.6), Vec2(301.0, 450.1), Vec2(303.1, 439.3), Vec2(305.2, 428.5), Vec2(309.1, 416.5), Vec2(313.1, 404.5), Vec2(318.9, 393.0), Vec2(324.6, 381.5), Vec2(329.1, 374.5), Vec2(333.6, 367.5), Vec2(341.6, 357.5), Vec2(349.6, 347.5), Vec2(352.7, 343.0), Vec2(355.7, 338.5), Vec2(359.5, 331.0), Vec2(363.3, 323.5), Vec2(365.7, 317.1), Vec2(368.1, 310.8), Vec2(370.5, 301.4), Vec2(372.9, 292.1), Vec2(374.5, 280.8), Vec2(376.0, 269.5), Vec2(376.5, 226.6), Vec2(376.9, 183.8), Vec2(395.2, 183.7), Vec2(412.9, 184.3), Vec2(413.6, 190.6), Vec2(414.2, 197.0), Vec2(413.7, 234.7), Vec2(413.2, 272.5), Vec2(411.2, 285.5), Vec2(409.1, 298.5), Vec2(407.0, 308.5), Vec2(404.8, 318.5), Vec2(401.3, 329.0), Vec2(397.9, 339.5), Vec2(392.4, 350.5), Vec2(387.0, 361.5), Vec2(378.8, 372.5), Vec2(370.6, 383.5), Vec2(365.4, 391.6), Vec2(360.1, 399.7), Vec2(355.2, 409.6), Vec2(350.2, 419.5), Vec2(347.2, 428.0), Vec2(344.2, 436.5), Vec2(343.2, 442.0), Vec2(342.1, 447.5), Vec2(341.5, 452.3), Vec2(340.9, 457.0), Vec2(338.9, 457.0), Vec2(336.9, 457.0), Vec2(337.5, 452.3), Vec2(338.1, 447.5), Vec2(339.7, 440.0), Vec2(341.2, 432.5), Vec2(344.1, 424.9), Vec2(346.9, 417.2), Vec2(351.9, 407.1), Vec2(357.0, 397.0), Vec2(360.7, 391.3), Vec2(364.3, 385.5), Vec2(372.6, 374.3), Vec2(380.9, 363.1), Vec2(385.4, 355.3), Vec2(389.9, 347.5), Vec2(394.0, 337.4), Vec2(398.0, 327.3), Vec2(401.0, 315.9), Vec2(404.0, 304.5), Vec2(406.0, 292.5), Vec2(407.9, 280.5), Vec2(409.0, 269.0), Vec2(410.0, 257.5), Vec2(410.0, 224.0), Vec2(410.0, 190.5), Vec2(409.2, 189.4), Vec2(408.5, 188.3), Vec2(394.8, 187.7), Vec2(381.0, 187.2), Vec2(381.0, 219.3), Vec2(381.0, 251.5), Vec2(380.0, 266.0), Vec2(378.9, 280.5), Vec2(376.5, 292.6), Vec2(374.0, 304.8), Vec2(370.9, 314.1), Vec2(367.8, 323.5), Vec2(362.7, 333.4), Vec2(357.7, 343.3), Vec2(351.6, 351.2), Vec2(345.5, 359.2), Vec2(339.3, 367.3), Vec2(333.0, 375.5), Vec2(331.1, 379.1), Vec2(329.3, 381.1), Vec2(323.2, 393.3), Vec2(317.1, 405.5), Vec2(313.1, 417.5), Vec2(309.1, 429.5), Vec2(307.1, 439.7), Vec2(305.0, 449.8), Vec2(305.0, 453.4), Vec2(305.0, 457.0), Vec2(303.0, 457.0), Vec2(301.0, 457.0), Vec2(301.0, 453.6)];
        PL.addShape(table, x, y, vs, 2);

        // left rail hole
        PL.addRamp(table, {x: 6.3, y: 21.3}, {x: 6.3, y: 19.5}, 2, 1);
        // right rail hole
        PL.addRamp(table, {x: 39.6, y: 21.3}, {x: 39.6, y: 19.5}, 2, 1);

        PL.addFlipper(table, 14.5, 3.82, -Math.PI / 2, PL.flipperIndex.left, 1);
        PL.addFlipper(table, 31.5, 3.8, Math.PI / 2, PL.flipperIndex.right, 1);
    },
    addMiddleLeft1 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer 1
        var vs = [Vec2(0, 330.1), Vec2(0, 241.0), Vec2(7.9, 241.0), Vec2(15.9, 241.0), Vec2(28.5, 250.1), Vec2(41.2, 259.3), Vec2(42.2, 261.9), Vec2(43.2, 264.5), Vec2(42.7, 270.5), Vec2(42.1, 276.5), Vec2(38.1, 321.0), Vec2(34.0, 365.5), Vec2(32.5, 382.3), Vec2(31.0, 399.2), Vec2(29.8, 401.3), Vec2(28.7, 403.5), Vec2(23.8, 410.1), Vec2(19.0, 416.8), Vec2(16.7, 417.3), Vec2(14.5, 417.9), Vec2(7.3, 418.6), Vec2(0, 419.3), Vec2(0, 330.1)];
        PL.addShape(table, x, y, vs, 1);
        // outer 2
        vs = [Vec2(0, 485.0), Vec2(0, 457.0), Vec2(3.0, 457.0), Vec2(6.0, 457.0), Vec2(13.3, 455.1), Vec2(20.5, 453.1), Vec2(25.0, 451.5), Vec2(29.4, 449.9), Vec2(33.5, 447.6), Vec2(37.5, 445.2), Vec2(42.6, 439.8), Vec2(47.6, 434.5), Vec2(50.5, 430.2), Vec2(53.3, 425.9), Vec2(54.9, 426.2), Vec2(56.5, 426.5), Vec2(56.2, 433.5), Vec2(55.9, 440.5), Vec2(62.9, 451.4), Vec2(69.9, 462.3), Vec2(71.7, 464.2), Vec2(73.4, 466.1), Vec2(72.1, 474.8), Vec2(70.8, 483.5), Vec2(69.9, 490.5), Vec2(69.1, 497.5), Vec2(69.0, 505.3), Vec2(69.0, 513.0), Vec2(64.0, 513.0), Vec2(59.0, 513.0), Vec2(59.0, 507.7), Vec2(59.0, 502.4), Vec2(57.8, 501.2), Vec2(56.6, 500.0), Vec2(44.0, 500.0), Vec2(31.4, 500.0), Vec2(30.2, 501.2), Vec2(29.0, 502.4), Vec2(29.0, 507.7), Vec2(29.0, 513.0), Vec2(14.5, 513.0), Vec2(0, 513.0), Vec2(0, 485.0)];
        PL.addShape(table, x, y, vs, 1);
        // island 1
        vs = [Vec2(83.3, 362.4), Vec2(76.0, 356.9), Vec2(76.0, 355.6), Vec2(76.0, 354.2), Vec2(87.8, 344.1), Vec2(99.5, 334.0), Vec2(100.6, 334.0), Vec2(101.8, 334.0), Vec2(102.8, 335.3), Vec2(103.8, 336.5), Vec2(105.4, 342.7), Vec2(107.0, 349.0), Vec2(109.0, 352.7), Vec2(111.0, 356.5), Vec2(112.2, 358.2), Vec2(113.5, 359.9), Vec2(109.5, 364.0), Vec2(105.6, 368.0), Vec2(98.0, 368.0), Vec2(90.5, 368.0), Vec2(83.3, 362.4)];
        PL.addShape(table, x, y, vs, 1);
        // island 2
        vs = [Vec2(161.5, 449.2), Vec2(156.5, 446.4), Vec2(148.0, 441.1), Vec2(139.5, 435.8), Vec2(137.8, 434.2), Vec2(136.0, 432.7), Vec2(136.0, 430.5), Vec2(136.0, 428.3), Vec2(143.3, 421.3), Vec2(150.5, 414.3), Vec2(152.8, 413.6), Vec2(155.1, 412.8), Vec2(156.4, 413.9), Vec2(157.8, 415.1), Vec2(164.6, 431.6), Vec2(171.4, 448.2), Vec2(170.3, 449.9), Vec2(169.3, 451.5), Vec2(167.9, 451.7), Vec2(166.5, 452.0), Vec2(161.5, 449.2)];
        PL.addShape(table, x, y, vs, 1);

        PL.addPopBumper(table, 5.5, 45.9, 1);
        PL.addPopBumper(table, 8.7, 37.4, 1);
        PL.addPopBumper(table, 14.4, 44.5, 1);

        // kickback
        PL.addBumperWall(table, {x: 4.3, y: 50}, {x: 4.3, y: 51}, 1, {x: 4.3, y: 40, strength: PL.kickStrength});
    },
    addMiddleLeft2 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer 1
        var vs = [Vec2(59.0, 492.7), Vec2(59.0, 472.5), Vec2(56.9, 465.0), Vec2(54.9, 457.5), Vec2(51.0, 449.5), Vec2(47.2, 441.5), Vec2(43.4, 436.7), Vec2(39.7, 431.9), Vec2(33.8, 427.5), Vec2(27.8, 423.0), Vec2(21.3, 421.0), Vec2(14.8, 419.0), Vec2(7.4, 419.0), Vec2(0, 419.0), Vec2(0, 330.0), Vec2(0, 241.0), Vec2(7.8, 241.0), Vec2(15.7, 241.0), Vec2(31.6, 264.3), Vec2(47.5, 287.5), Vec2(47.4, 289.5), Vec2(47.2, 291.6), Vec2(43.7, 297.2), Vec2(40.1, 302.7), Vec2(35.6, 312.1), Vec2(31.1, 321.5), Vec2(28.4, 331.0), Vec2(25.6, 340.5), Vec2(25.5, 355.0), Vec2(25.5, 369.5), Vec2(28.3, 380.0), Vec2(31.2, 390.5), Vec2(35.4, 399.0), Vec2(39.6, 407.5), Vec2(42.3, 411.5), Vec2(45.0, 415.5), Vec2(53.0, 425.5), Vec2(61.0, 435.5), Vec2(62.5, 437.6), Vec2(64.1, 439.7), Vec2(66.3, 445.6), Vec2(68.5, 451.5), Vec2(68.8, 482.3), Vec2(69.1, 513.0), Vec2(64.1, 513.0), Vec2(59.0, 513.0), Vec2(59.0, 492.7)];
        PL.addShape(table, x, y, vs, 1);
        // outer 2
        vs = [Vec2(0, 484.9), Vec2(0, 456.8), Vec2(3.0, 457.4), Vec2(6.1, 458.0), Vec2(10.5, 461.0), Vec2(14.9, 464.1), Vec2(17.4, 467.3), Vec2(19.8, 470.5), Vec2(22.3, 475.5), Vec2(24.7, 480.5), Vec2(26.4, 487.0), Vec2(28.1, 493.5), Vec2(28.7, 503.3), Vec2(29.3, 513.0), Vec2(14.6, 513.0), Vec2(0, 513.0), Vec2(0, 484.9)];
        PL.addShape(table, x, y, vs, 1);
        // island 1
        vs = [Vec2(84.8, 412.3), Vec2(81.3, 408.5), Vec2(78.1, 403.8), Vec2(75.0, 399.1), Vec2(75.0, 397.2), Vec2(72.1, 391.4), Vec2(69.2, 385.5), Vec2(68.1, 381.5), Vec2(67.0, 377.5), Vec2(67.0, 372.9), Vec2(67.0, 368.4), Vec2(68.6, 362.9), Vec2(70.2, 357.5), Vec2(73.3, 351.7), Vec2(76.3, 345.9), Vec2(81.7, 341.0), Vec2(87.0, 336.1), Vec2(92.5, 333.5), Vec2(98.1, 330.9), Vec2(99.0, 331.5), Vec2(100.0, 332.1), Vec2(100.0, 333.1), Vec2(93.1, 345.5), Vec2(86.1, 357.1), Vec2(84.0, 362.8), Vec2(81.8, 368.5), Vec2(82.2, 375.0), Vec2(82.6, 381.5), Vec2(85.9, 388.0), Vec2(89.1, 394.5), Vec2(91.1, 396.6), Vec2(93.0, 398.6), Vec2(93.0, 406.1), Vec2(93.0, 413.6), Vec2(91.8, 414.8), Vec2(90.6, 416.0), Vec2(89.5, 416.0), Vec2(88.4, 416.0), Vec2(84.8, 412.3)];
        PL.addShape(table, x, y, vs, 1);
    },
    addMiddleLeft3 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer 1
        var vs = [Vec2(0, 330.0), Vec2(0, 241.0), Vec2(7.8, 241.0), Vec2(15.5, 241.0), Vec2(25.3, 250.8), Vec2(35.1, 260.7), Vec2(34.7, 272.6), Vec2(34.4, 284.5), Vec2(22.6, 327.5), Vec2(10.8, 370.5), Vec2(9.4, 379.0), Vec2(8.0, 387.5), Vec2(8.0, 401.6), Vec2(8.0, 415.7), Vec2(6.2, 417.3), Vec2(4.3, 419.0), Vec2(2.2, 419.0), Vec2(0, 419.0), Vec2(0, 330.0)];
        PL.addShape(table, x, y, vs, 1);
        // outer 2
        vs = [Vec2(0, 485.2), Vec2(0, 457.3), Vec2(6.8, 456.1), Vec2(13.7, 454.9), Vec2(18.7, 452.6), Vec2(23.7, 450.2), Vec2(30.3, 444.1), Vec2(36.8, 437.9), Vec2(42.0, 422.0), Vec2(47.1, 406.0), Vec2(50.6, 406.0), Vec2(54.0, 406.0), Vec2(55.2, 403.5), Vec2(56.3, 401.0), Vec2(57.3, 401.0), Vec2(57.7, 402.8), Vec2(57.2, 404.5), Vec2(44.6, 436.3), Vec2(32.0, 468.0), Vec2(31.0, 474.3), Vec2(29.9, 480.5), Vec2(29.3, 496.8), Vec2(28.7, 513.0), Vec2(14.3, 513.0), Vec2(0, 513.0), Vec2(0, 485.2)];
        PL.addShape(table, x, y, vs, 1);
        // island 1
        vs = [Vec2(59.1, 511.3), Vec2(59.2, 509.5), Vec2(62.8, 495.5), Vec2(66.5, 481.5), Vec2(67.5, 481.5), Vec2(68.5, 481.5), Vec2(68.8, 497.3), Vec2(69.1, 513.0), Vec2(64.0, 513.0), Vec2(59.0, 513.0), Vec2(59.1, 511.3)];
        PL.addShape(table, x, y, vs, 1);

        PL.addFlipper(table, 5, 38.9, (-Math.PI * (11/16)), PL.flipperIndex.midLeft, 1);
    },
    addMiddleRight1 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer 1
        var vs = [Vec2(453.7, 417.2), Vec2(452.1, 415.4), Vec2(451.8, 353.5), Vec2(451.4, 291.5), Vec2(448.2, 281.9), Vec2(445.0, 272.3), Vec2(441.5, 264.6), Vec2(438.0, 256.9), Vec2(438.0, 252.1), Vec2(438.0, 247.4), Vec2(441.3, 244.2), Vec2(444.6, 241.0), Vec2(452.3, 241.0), Vec2(460.0, 241.0), Vec2(460.0, 330.0), Vec2(460.0, 419.0), Vec2(457.7, 419.0), Vec2(455.3, 419.0), Vec2(453.7, 417.2)];
        PL.addShape(table, x, y, vs, 1);
        // outer 2
        vs = [Vec2(417.7, 509.5), Vec2(416.0, 505.9), Vec2(417.4, 505.0), Vec2(418.8, 505.0), Vec2(419.4, 506.5), Vec2(419.9, 508.0), Vec2(422.4, 508.0), Vec2(424.8, 508.0), Vec2(433.4, 504.0), Vec2(442.0, 500.1), Vec2(442.0, 496.2), Vec2(442.0, 492.3), Vec2(420.0, 455.4), Vec2(398.0, 418.5), Vec2(398.0, 376.7), Vec2(398.0, 334.9), Vec2(399.6, 334.3), Vec2(401.2, 333.7), Vec2(410.0, 342.5), Vec2(418.9, 351.4), Vec2(419.2, 386.9), Vec2(419.5, 422.5), Vec2(422.8, 430.7), Vec2(426.1, 438.9), Vec2(432.1, 444.5), Vec2(438.2, 450.0), Vec2(442.1, 452.1), Vec2(446.1, 454.1), Vec2(453.0, 455.6), Vec2(460.0, 457.2), Vec2(460.0, 485.1), Vec2(460.0, 513.0), Vec2(439.7, 513.0), Vec2(419.4, 513.0), Vec2(417.7, 509.5)];
        PL.addShape(table, x, y, vs, 1);

        // drop targets
        PL.addWall(table, 
            {x: 39.6, y: 41.3},
            {x: 39.6, y: 39.1}, 
            1, {group: "todo", id: 0}
        );
        PL.addWall(table, 
            {x: 39.6, y: 38.4},
            {x: 39.6, y: 36.5},
            1, {group: "todo", id: 1}
        );
        PL.addWall(table, 
            {x: 39.6, y: 35.7},
            {x: 39.6, y: 33.7},
            1, {group: "todo", id: 2}
        );

        PL.addFlipper(table, 42.4, 49.3, (Math.PI * (11/16)), PL.flipperIndex.midRight, 1);
    },
    addMiddleRight2 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer 1
        var vs = [Vec2(456.8, 416.3), Vec2(454.2, 413.5), Vec2(452.1, 408.5), Vec2(450.1, 403.5), Vec2(440.1, 368.0), Vec2(430.1, 332.5), Vec2(429.5, 329.7), Vec2(428.8, 327.0), Vec2(436.4, 313.7), Vec2(444.0, 300.4), Vec2(444.0, 270.7), Vec2(444.0, 241.0), Vec2(452.0, 241.0), Vec2(460.0, 241.0), Vec2(460.0, 330.0), Vec2(460.0, 419.0), Vec2(456.8, 416.3)];
        PL.addShape(table, x, y, vs, 1);
        // outer 2
        vs = [Vec2(419.0, 510.4), Vec2(418.0, 507.9), Vec2(409.1, 486.3), Vec2(400.2, 465.5), Vec2(393.6, 450.5), Vec2(387.0, 435.6), Vec2(388.4, 434.0), Vec2(389.8, 434.0), Vec2(391.0, 435.5), Vec2(392.2, 437.0), Vec2(394.1, 437.0), Vec2(395.9, 437.0), Vec2(401.5, 434.4), Vec2(407.0, 431.7), Vec2(407.8, 430.6), Vec2(408.5, 429.5), Vec2(409.5, 426.5), Vec2(410.5, 423.5), Vec2(412.5, 423.5), Vec2(414.4, 423.5), Vec2(415.8, 427.5), Vec2(417.2, 431.5), Vec2(419.8, 437.3), Vec2(422.4, 443.0), Vec2(426.9, 446.6), Vec2(431.5, 450.1), Vec2(437.6, 453.1), Vec2(443.8, 456.2), Vec2(451.9, 456.7), Vec2(460.0, 457.3), Vec2(460.0, 485.1), Vec2(460.0, 513.0), Vec2(440.0, 513.0), Vec2(420.0, 513.0), Vec2(419.0, 510.4)];
        PL.addShape(table, x, y, vs, 1);

        PL.addFlipper(table, 39.6, 42.2, (Math.PI * (11/16)), PL.flipperIndex.midRight, 1);
    },
    addMiddleRight3 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer 1
        var vs = [Vec2(458.5, 403.3), Vec2(457.8, 401.5), Vec2(452.0, 386.0), Vec2(446.2, 370.5), Vec2(435.6, 347.7), Vec2(425.1, 324.9), Vec2(422.0, 293.8), Vec2(418.8, 262.7), Vec2(423.2, 258.3), Vec2(427.5, 253.9), Vec2(436.1, 247.4), Vec2(444.7, 241.0), Vec2(452.4, 241.0), Vec2(460.0, 241.0), Vec2(460.0, 323.0), Vec2(460.0, 405.0), Vec2(458.5, 403.3)];
        PL.addShape(table, x, y, vs, 1);
        // outer 2
        vs = [Vec2(419.0, 510.8), Vec2(418.1, 508.5), Vec2(411.9, 491.6), Vec2(405.7, 474.7), Vec2(407.2, 473.6), Vec2(408.7, 472.5), Vec2(419.4, 469.5), Vec2(430.0, 466.5), Vec2(430.0, 458.2), Vec2(430.0, 449.9), Vec2(428.3, 439.9), Vec2(426.7, 429.9), Vec2(427.8, 428.8), Vec2(428.9, 427.7), Vec2(430.3, 428.3), Vec2(431.7, 428.8), Vec2(433.3, 437.2), Vec2(435.0, 445.5), Vec2(438.4, 448.7), Vec2(441.8, 451.9), Vec2(448.5, 453.9), Vec2(455.2, 456.0), Vec2(457.6, 456.0), Vec2(460.0, 456.0), Vec2(460.0, 484.5), Vec2(460.0, 513.0), Vec2(440.0, 513.0), Vec2(420.0, 513.0), Vec2(419.0, 510.8)];
        PL.addShape(table, x, y, vs, 1);
        // island 1
        vs = [Vec2(312.7, 454.3), Vec2(312.0, 453.7), Vec2(312.0, 451.9), Vec2(312.0, 450.1), Vec2(323.7, 437.1), Vec2(335.3, 424.0), Vec2(336.6, 424.0), Vec2(338.0, 424.0), Vec2(345.1, 433.4), Vec2(352.2, 442.9), Vec2(351.8, 444.7), Vec2(351.5, 446.5), Vec2(335.5, 450.7), Vec2(319.5, 454.8), Vec2(316.4, 454.9), Vec2(313.3, 455.0), Vec2(312.7, 454.3)];
        PL.addShape(table, x, y, vs, 1);
        // island 2
        vs = [Vec2(376.1, 381.1), Vec2(373.7, 380.5), Vec2(372.7, 378.9), Vec2(371.7, 377.3), Vec2(380.4, 352.9), Vec2(389.0, 328.6), Vec2(390.0, 327.6), Vec2(390.9, 326.7), Vec2(392.3, 327.3), Vec2(393.8, 327.9), Vec2(399.9, 353.2), Vec2(406.0, 378.5), Vec2(406.0, 380.3), Vec2(406.0, 382.0), Vec2(392.3, 381.9), Vec2(378.5, 381.8), Vec2(376.1, 381.1)];
        PL.addShape(table, x, y, vs, 1);

        PL.addPopBumper(table, 33.7, 45.6, 1);
        PL.addPopBumper(table, 39.5, 38.5, 1);
        PL.addPopBumper(table, 42.8, 47.0, 1);
    },
    addOutfield1 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer left
        var vs = [Vec2(0, 677.5), Vec2(0, 513.0), Vec2(14.5, 513.0), Vec2(28.9, 513.0), Vec2(29.3, 628.3), Vec2(29.6, 743.5), Vec2(31.3, 752.3), Vec2(33.1, 761.0), Vec2(35.6, 769.8), Vec2(38.1, 778.5), Vec2(42.5, 787.9), Vec2(47.0, 797.2), Vec2(51.4, 803.4), Vec2(55.8, 809.6), Vec2(62.7, 816.0), Vec2(69.6, 822.5), Vec2(76.1, 826.3), Vec2(82.5, 830.0), Vec2(92.0, 833.1), Vec2(101.5, 836.2), Vec2(112.5, 836.8), Vec2(123.5, 837.5), Vec2(123.8, 839.8), Vec2(124.1, 842.0), Vec2(62.1, 842.0), Vec2(0, 842.0), Vec2(0, 677.5)];
        PL.addShape(table, x, y, vs, 1);
        // outer right
        vs = [Vec2(336.2, 839.8), Vec2(336.5, 837.5), Vec2(347.5, 836.8), Vec2(358.5, 836.2), Vec2(368.0, 833.1), Vec2(377.5, 830.0), Vec2(383.9, 826.3), Vec2(390.4, 822.5), Vec2(397.3, 816.0), Vec2(404.2, 809.6), Vec2(408.6, 803.4), Vec2(413.0, 797.2), Vec2(417.5, 787.9), Vec2(421.9, 778.5), Vec2(424.4, 769.7), Vec2(426.9, 761.0), Vec2(428.7, 751.7), Vec2(430.5, 742.5), Vec2(430.5, 653.5), Vec2(430.5, 564.5), Vec2(429.2, 553.5), Vec2(428.0, 542.5), Vec2(426.6, 537.5), Vec2(425.2, 532.5), Vec2(422.6, 523.2), Vec2(420.0, 513.9), Vec2(440.0, 513.0), Vec2(460.0, 513.0), Vec2(460.0, 677.5), Vec2(460.0, 842.0), Vec2(397.9, 842.0), Vec2(335.9, 842.0), Vec2(336.2, 839.8)];
        PL.addShape(table, x, y, vs, 1);
        // left island 1
        vs = [Vec2(59.7, 599.4), Vec2(59.0, 598.7), Vec2(59.0, 555.8), Vec2(59.0, 513.0), Vec2(64.0, 513.0), Vec2(69.0, 513.0), Vec2(69.0, 518.4), Vec2(69.0, 523.7), Vec2(72.7, 531.1), Vec2(76.3, 538.5), Vec2(88.2, 550.1), Vec2(100.0, 561.8), Vec2(100.0, 563.2), Vec2(100.0, 564.7), Vec2(81.4, 582.1), Vec2(62.9, 599.5), Vec2(61.6, 599.8), Vec2(60.4, 600.1), Vec2(59.7, 599.4)];
        PL.addShape(table, x, y, vs, 1);
        // left island 2
        vs = [Vec2(144.5, 805.8), Vec2(140.5, 804.8), Vec2(132.7, 802.9), Vec2(124.9, 801.0), Vec2(116.5, 797.1), Vec2(108.2, 793.2), Vec2(100.3, 787.5), Vec2(92.5, 781.7), Vec2(88.7, 778.1), Vec2(84.8, 774.5), Vec2(79.9, 768.4), Vec2(75.1, 762.3), Vec2(71.5, 754.9), Vec2(68.0, 747.6), Vec2(68.0, 746.2), Vec2(68.0, 744.8), Vec2(69.4, 745.3), Vec2(70.9, 745.9), Vec2(90.3, 758.8), Vec2(109.7, 771.8), Vec2(115.9, 775.0), Vec2(122.2, 778.1), Vec2(129.8, 780.9), Vec2(137.5, 783.7), Vec2(145.5, 785.4), Vec2(153.5, 787.1), Vec2(155.8, 787.6), Vec2(158.0, 788.0), Vec2(158.0, 795.9), Vec2(158.0, 803.7), Vec2(156.2, 805.3), Vec2(154.3, 807.0), Vec2(151.4, 806.9), Vec2(148.5, 806.8), Vec2(144.5, 805.8)];
        PL.addShape(table, x, y, vs, 1);
        // left island 3
        vs = [Vec2(203.8, 734.3), Vec2(201.2, 731.5), Vec2(199.4, 728.0), Vec2(197.6, 724.5), Vec2(197.6, 717.5), Vec2(197.6, 710.5), Vec2(200.3, 705.3), Vec2(203.0, 700.2), Vec2(212.1, 692.1), Vec2(221.2, 684.0), Vec2(222.4, 684.0), Vec2(223.5, 684.0), Vec2(232.9, 693.4), Vec2(242.3, 702.8), Vec2(241.8, 704.2), Vec2(241.2, 705.6), Vec2(224.7, 721.3), Vec2(208.3, 737.0), Vec2(207.3, 737.0), Vec2(203.8, 734.3)];
        PL.addShape(table, x, y, vs, 1);
        // right island 1
        vs = [Vec2(370.9, 735.8), Vec2(365.8, 729.5), Vec2(360.7, 724.9), Vec2(355.5, 720.3), Vec2(356.9, 718.6), Vec2(358.2, 717.0), Vec2(359.3, 717.0), Vec2(360.3, 717.0), Vec2(376.7, 707.4), Vec2(393.1, 697.8), Vec2(394.5, 698.4), Vec2(396.0, 698.9), Vec2(396.0, 701.2), Vec2(396.0, 703.5), Vec2(393.6, 712.6), Vec2(391.1, 721.6), Vec2(388.7, 727.2), Vec2(386.2, 732.9), Vec2(382.0, 737.4), Vec2(377.7, 742.0)];
        PL.addShape(table, x, y, vs, 1);
        // right island 2
        vs = [Vec2(296.0, 806.1), Vec2(295.0, 804.1), Vec2(295.0, 774.1), Vec2(295.0, 744.0), Vec2(297.0, 742.0), Vec2(299.0, 740.0), Vec2(302.9, 744.2), Vec2(305.3, 748.5), Vec2(310.4, 753.9), Vec2(315.5, 759.2), Vec2(321.7, 763.6), Vec2(327.8, 768.0), Vec2(334.4, 771.0), Vec2(340.9, 774.0), Vec2(347.7, 775.8), Vec2(354.5, 777.7), Vec2(354.8, 779.6), Vec2(355.1, 781.5), Vec2(349.8, 787.4), Vec2(344.5, 793.3)];
        PL.addShape(table, x, y, vs, 1);
        // left ramp base
        vs = [Vec2(139.4, 749.7), Vec2(122.2, 740.5), Vec2(96.6, 724.5), Vec2(70.9, 708.6), Vec2(67.5, 705.7), Vec2(64.0, 702.9), Vec2(64.0, 696.4), Vec2(64.0, 689.9), Vec2(66.2, 684.7), Vec2(68.4, 679.5), Vec2(74.9, 669.5), Vec2(81.5, 659.5), Vec2(83.3, 656.0), Vec2(85.2, 652.5), Vec2(101.3, 628.5), Vec2(117.5, 604.6), Vec2(120.9, 603.7), Vec2(124.3, 602.9), Vec2(125.8, 604.3), Vec2(127.2, 605.8), Vec2(107.1, 657.5), Vec2(87.0, 709.1), Vec2(87.6, 710.1), Vec2(88.2, 711.2), Vec2(101.8, 720.1), Vec2(115.4, 729.1), Vec2(117.4, 728.8), Vec2(119.4, 728.5), Vec2(148.5, 682.5), Vec2(177.6, 636.5), Vec2(179.3, 636.2), Vec2(181.0, 635.9), Vec2(181.0, 637.4), Vec2(181.0, 639.0), Vec2(168.6, 660.8), Vec2(156.2, 682.6), Vec2(154.5, 688.1), Vec2(152.9, 693.5), Vec2(152.3, 703.0), Vec2(151.8, 712.5), Vec2(153.4, 719.3), Vec2(155.0, 726.0), Vec2(158.1, 732.3), Vec2(161.2, 738.5), Vec2(166.6, 745.1), Vec2(171.9, 751.8), Vec2(170.2, 753.3), Vec2(168.5, 754.9), Vec2(164.0, 756.9), Vec2(159.5, 759.0), Vec2(158.0, 758.9), Vec2(156.5, 758.9), Vec2(139.4, 749.7)];
        PL.addShape(table, x, y, vs, 1);
        // right ramp base
        vs = [Vec2(332.2, 688.3), Vec2(329.2, 683.5), Vec2(306.4, 657.7), Vec2(283.5, 631.9), Vec2(278.6, 626.5), Vec2(273.8, 621.1), Vec2(274.3, 619.8), Vec2(274.8, 618.5), Vec2(304.6, 601.6), Vec2(334.5, 584.7), Vec2(336.2, 583.2), Vec2(337.9, 581.8), Vec2(339.4, 582.3), Vec2(340.9, 582.9), Vec2(344.8, 590.7), Vec2(348.7, 598.5), Vec2(352.3, 607.5), Vec2(355.8, 616.5), Vec2(357.4, 622.5), Vec2(359.0, 628.5), Vec2(360.0, 632.8), Vec2(361.1, 637.0), Vec2(378.7, 637.0), Vec2(396.2, 637.0), Vec2(395.7, 610.3), Vec2(395.3, 583.5), Vec2(393.0, 571.1), Vec2(390.8, 558.7), Vec2(387.8, 548.6), Vec2(384.9, 538.5), Vec2(379.9, 528.1), Vec2(374.9, 517.6), Vec2(375.4, 516.3), Vec2(375.9, 515.0), Vec2(377.4, 515.0), Vec2(378.8, 515.0), Vec2(381.8, 520.3), Vec2(384.7, 525.5), Vec2(388.3, 534.5), Vec2(391.9, 543.6), Vec2(395.0, 556.0), Vec2(398.0, 568.5), Vec2(399.5, 581.5), Vec2(401.0, 594.5), Vec2(401.0, 618.2), Vec2(401.0, 641.9), Vec2(395.1, 648.7), Vec2(389.3, 655.5), Vec2(383.4, 661.0), Vec2(377.5, 666.6), Vec2(370.5, 672.2), Vec2(363.5, 677.8), Vec2(356.3, 682.3), Vec2(349.1, 686.9), Vec2(343.1, 690.0), Vec2(337.1, 693.0), Vec2(336.1, 693.0), Vec2(332.2, 688.3)];
        PL.addShape(table, x, y, vs, 1);
        // seperator 1
        vs = [Vec2(188.1, 807.6), Vec2(186.9, 806.2), Vec2(187.2, 798.0), Vec2(187.5, 789.7), Vec2(189.2, 788.7), Vec2(190.8, 787.6), Vec2(192.4, 788.9), Vec2(194.0, 790.2), Vec2(194.0, 798.4), Vec2(194.0, 806.6), Vec2(192.8, 807.8), Vec2(191.6, 809.0), Vec2(190.4, 809.0), Vec2(189.2, 809.0), Vec2(188.1, 807.6)];
        PL.addShape(table, x, y, vs, 1);
        // sep 2
        vs = [Vec2(224.1, 807.6), Vec2(222.9, 806.2), Vec2(223.2, 798.0), Vec2(223.5, 789.7), Vec2(225.2, 788.7), Vec2(226.8, 787.6), Vec2(228.4, 788.9), Vec2(230.0, 790.2), Vec2(230.0, 798.4), Vec2(230.0, 806.6), Vec2(228.8, 807.8), Vec2(227.6, 809.0), Vec2(226.4, 809.0), Vec2(225.2, 809.0), Vec2(224.1, 807.6)];
        PL.addShape(table, x, y, vs, 1);
        // sep 3
        vs = [Vec2(260.1, 807.6), Vec2(258.9, 806.2), Vec2(259.2, 798.0), Vec2(259.5, 789.7), Vec2(261.2, 788.7), Vec2(262.8, 787.6), Vec2(264.4, 788.9), Vec2(266.0, 790.2), Vec2(266.0, 798.4), Vec2(266.0, 806.6), Vec2(264.8, 807.8), Vec2(263.6, 809.0), Vec2(262.4, 809.0), Vec2(261.2, 809.0), Vec2(260.1, 807.6)];
        PL.addShape(table, x, y, vs, 1);

        PL.addPopBumper(table, 22.2, 71.7, 1);
        PL.addPopBumper(table, 31.1, 75.0, 1);
        PL.addPopBumper(table, 29.6, 65.5, 1);

        // drop targets
        PL.addWall(table, 
            {x: 27.4, y: 61.6},
            {x: 29.2, y: 60.5}, 
            1, {group: "todo", id: 0}
        );
        PL.addWall(table, 
            {x: 29.6, y: 60.3},
            {x: 31.5, y: 59.2},
            1, {group: "todo", id: 1}
        );
        PL.addWall(table, 
            {x: 31.9, y: 59.1},
            {x: 33.7, y: 57.9},
            1, {group: "todo", id: 2}
        );

        // left ramp up
        PL.addRamp(table, {x: 11.5, y: 69.1}, {x: 10, y: 71.8}, 1, 2);
        // left ramp down
        PL.addRamp(table, {x: 12.8, y: 63.2}, {x: 15.2, y: 65.2}, 2, 1);
        // right ramp up
        PL.addRamp(table, {x: 37, y: 61.3}, {x: 38.4, y: 65.1}, 1, 2);
        // right ramp down
        PL.addRamp(table, {x: 34.7, y: 55.7}, {x: 37, y: 54.6}, 2, 1);

        PL.addOutfield1Rails(table, x, y);
    },
    addOutfield1Rails : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // left rail side
        var vs = [Vec2(122.5, 794.1), Vec2(115.5, 792.4), Vec2(113.0, 791.0), Vec2(110.5, 789.7), Vec2(103.5, 784.5), Vec2(96.5, 779.2), Vec2(91.3, 773.9), Vec2(86.2, 768.5), Vec2(82.2, 762.7), Vec2(78.1, 756.8), Vec2(75.1, 749.7), Vec2(72.2, 742.5), Vec2(70.4, 734.1), Vec2(68.7, 725.6), Vec2(69.3, 714.1), Vec2(69.8, 702.5), Vec2(70.9, 699.5), Vec2(72.1, 696.5), Vec2(96.8, 650.3), Vec2(121.5, 604.0), Vec2(124.0, 604.0), Vec2(126.5, 604.0), Vec2(147.8, 625.5), Vec2(169.0, 647.0), Vec2(169.0, 651.8), Vec2(169.0, 656.6), Vec2(148.9, 689.5), Vec2(128.7, 722.5), Vec2(128.2, 724.3), Vec2(127.8, 726.0), Vec2(123.6, 731.4), Vec2(120.8, 736.9), Vec2(121.9, 742.2), Vec2(123.1, 747.5), Vec2(124.7, 750.0), Vec2(126.4, 752.5), Vec2(127.6, 753.7), Vec2(128.8, 755.0), Vec2(131.5, 756.0), Vec2(134.1, 757.0), Vec2(138.8, 757.0), Vec2(143.5, 757.0), Vec2(147.8, 755.0), Vec2(152.0, 753.1), Vec2(159.9, 744.8), Vec2(167.7, 736.5), Vec2(179.3, 724.0), Vec2(191.0, 711.5), Vec2(191.0, 652.7), Vec2(191.0, 593.9), Vec2(189.3, 587.2), Vec2(187.7, 580.5), Vec2(149.6, 542.3), Vec2(111.5, 504.0), Vec2(88.3, 480.3), Vec2(65.1, 456.5), Vec2(32.6, 423.5), Vec2(0.1, 390.5), Vec2(0.1, 387.0), Vec2(0.1, 383.5), Vec2(14.8, 399.0), Vec2(29.5, 414.5), Vec2(34.4, 419.0), Vec2(39.2, 423.5), Vec2(64.9, 449.4), Vec2(90.5, 475.2), Vec2(131.0, 515.9), Vec2(171.5, 556.5), Vec2(183.7, 569.5), Vec2(196.0, 582.5), Vec2(196.0, 647.3), Vec2(196.0, 712.1), Vec2(175.5, 734.0), Vec2(155.0, 755.8), Vec2(149.8, 758.4), Vec2(144.6, 761.0), Vec2(138.8, 761.0), Vec2(132.9, 761.0), Vec2(129.7, 759.6), Vec2(126.5, 758.2), Vec2(123.8, 755.4), Vec2(121.1, 752.5), Vec2(119.4, 748.5), Vec2(117.8, 744.6), Vec2(117.7, 738.8), Vec2(117.7, 733.0), Vec2(122.8, 724.6), Vec2(128.0, 716.1), Vec2(128.0, 715.1), Vec2(128.8, 713.8), Vec2(129.6, 713.5), Vec2(148.4, 682.7), Vec2(167.2, 652.0), Vec2(166.8, 651.1), Vec2(166.5, 650.2), Vec2(146.4, 629.6), Vec2(126.2, 608.9), Vec2(120.6, 623.2), Vec2(115.1, 637.5), Vec2(101.0, 673.5), Vec2(86.8, 709.5), Vec2(84.8, 714.9), Vec2(82.8, 720.4), Vec2(82.3, 729.0), Vec2(81.7, 737.7), Vec2(83.4, 745.3), Vec2(85.1, 753.0), Vec2(88.6, 759.9), Vec2(92.0, 766.8), Vec2(98.4, 773.7), Vec2(104.8, 780.6), Vec2(111.0, 784.8), Vec2(117.2, 788.9), Vec2(124.7, 790.6), Vec2(132.3, 792.3), Vec2(140.4, 791.7), Vec2(148.6, 791.1), Vec2(155.0, 788.8), Vec2(161.5, 786.4), Vec2(167.0, 782.7), Vec2(172.5, 779.1), Vec2(175.8, 775.8), Vec2(179.2, 772.5), Vec2(202.1, 750.7), Vec2(225.0, 728.8), Vec2(225.0, 647.8), Vec2(225.0, 566.7), Vec2(185.1, 526.6), Vec2(145.2, 486.5), Vec2(100.8, 441.1), Vec2(56.5, 395.8), Vec2(28.2, 366.6), Vec2(-0.1, 337.5), Vec2(2.5, 337.2), Vec2(5.1, 336.9), Vec2(40.3, 372.7), Vec2(75.4, 408.5), Vec2(106.5, 439.6), Vec2(137.5, 470.7), Vec2(164.0, 497.6), Vec2(190.5, 524.5), Vec2(193.7, 528.3), Vec2(196.9, 532.0), Vec2(206.1, 539.8), Vec2(213.5, 547.6), Vec2(221.8, 556.7), Vec2(230.0, 565.9), Vec2(230.0, 647.7), Vec2(230.0, 729.5), Vec2(204.4, 754.0), Vec2(178.9, 778.5), Vec2(173.8, 782.7), Vec2(168.6, 786.9), Vec2(162.5, 789.9), Vec2(156.4, 792.9), Vec2(149.9, 794.5), Vec2(143.5, 796.0), Vec2(136.5, 795.9), Vec2(129.5, 795.9), Vec2(122.5, 794.1)];
        //PL.addShape(table, x, y, vs, 2);
        // right rail side
        vs = [Vec2(389.1, 680.4), Vec2(383.7, 679.0), Vec2(378.9, 676.3), Vec2(374.0, 673.7), Vec2(368.6, 668.0), Vec2(363.2, 662.4), Vec2(360.7, 657.4), Vec2(358.2, 652.5), Vec2(356.6, 647.6), Vec2(354.9, 642.7), Vec2(354.0, 633.1), Vec2(353.1, 623.4), Vec2(349.9, 614.5), Vec2(346.8, 605.5), Vec2(340.7, 593.7), Vec2(334.6, 581.9), Vec2(326.3, 569.2), Vec2(318.1, 556.6), Vec2(347.7, 536.2), Vec2(376.9, 516.0), Vec2(377.9, 516.0), Vec2(380.5, 519.8), Vec2(382.3, 523.5), Vec2(382.9, 524.0), Vec2(383.5, 524.5), Vec2(387.2, 536.0), Vec2(390.9, 547.5), Vec2(392.5, 553.9), Vec2(394.1, 560.2), Vec2(396.1, 570.8), Vec2(398.0, 581.4), Vec2(399.5, 595.7), Vec2(401.0, 609.9), Vec2(401.0, 622.3), Vec2(401.0, 634.6), Vec2(402.4, 636.6), Vec2(403.9, 638.5), Vec2(405.7, 637.4), Vec2(407.5, 636.3), Vec2(408.0, 510.2), Vec2(408.5, 384.1), Vec2(434.0, 357.9), Vec2(459.5, 331.6), Vec2(459.8, 334.4), Vec2(460.2, 337.3), Vec2(446.9, 350.9), Vec2(433.6, 364.5), Vec2(422.8, 375.6), Vec2(412.0, 386.6), Vec2(412.0, 511.9), Vec2(412.0, 637.1), Vec2(409.5, 639.5), Vec2(407.1, 642.0), Vec2(404.5, 642.0), Vec2(402.0, 642.0), Vec2(400.1, 640.1), Vec2(398.1, 638.1), Vec2(397.6, 634.8), Vec2(397.1, 631.5), Vec2(396.6, 614.5), Vec2(396.0, 597.5), Vec2(394.1, 583.9), Vec2(392.1, 570.2), Vec2(390.5, 563.4), Vec2(389.0, 556.5), Vec2(387.4, 550.3), Vec2(385.7, 544.0), Vec2(384.0, 541.0), Vec2(384.0, 538.0), Vec2(380.8, 530.0), Vec2(377.5, 522.0), Vec2(376.4, 522.0), Vec2(375.3, 522.0), Vec2(349.9, 539.6), Vec2(324.5, 557.2), Vec2(324.6, 558.3), Vec2(324.7, 559.5), Vec2(331.9, 570.5), Vec2(339.2, 581.5), Vec2(345.5, 594.5), Vec2(351.9, 607.5), Vec2(354.6, 615.5), Vec2(357.2, 623.5), Vec2(358.1, 633.1), Vec2(358.9, 642.6), Vec2(361.0, 648.3), Vec2(363.1, 654.0), Vec2(367.1, 659.9), Vec2(371.1, 665.9), Vec2(374.8, 668.8), Vec2(378.5, 671.7), Vec2(382.5, 673.7), Vec2(386.5, 675.7), Vec2(391.9, 677.0), Vec2(397.2, 678.3), Vec2(406.4, 677.7), Vec2(415.5, 677.2), Vec2(421.0, 675.1), Vec2(426.6, 673.0), Vec2(430.7, 670.0), Vec2(434.8, 667.0), Vec2(437.9, 662.2), Vec2(440.9, 657.5), Vec2(442.5, 651.5), Vec2(444.0, 645.6), Vec2(444.0, 522.8), Vec2(444.0, 400.1), Vec2(452.0, 391.9), Vec2(460.0, 383.8), Vec2(460.0, 386.5), Vec2(460.0, 389.2), Vec2(454.0, 395.9), Vec2(448.0, 402.7), Vec2(448.0, 523.8), Vec2(448.0, 644.9), Vec2(446.9, 650.2), Vec2(445.8, 655.5), Vec2(443.2, 660.8), Vec2(440.6, 666.1), Vec2(436.0, 670.4), Vec2(431.3, 674.8), Vec2(427.2, 676.9), Vec2(423.1, 679.0), Vec2(416.8, 680.5), Vec2(410.5, 682.0), Vec2(402.5, 681.9), Vec2(394.5, 681.9), Vec2(389.1, 680.4)];
        //PL.addShape(table, x, y, vs, 2);

        // left rail straight
        vs = [Vec2(141.0, 800.4), Vec2(134.5, 799.0), Vec2(130.1, 797.5), Vec2(125.6, 796.0), Vec2(122.9, 796.0), Vec2(120.1, 796.0), Vec2(110.8, 789.8), Vec2(101.6, 783.5), Vec2(94.6, 777.0), Vec2(87.7, 770.5), Vec2(82.9, 763.7), Vec2(78.1, 756.9), Vec2(75.1, 749.7), Vec2(72.2, 742.5), Vec2(70.4, 734.1), Vec2(68.7, 725.6), Vec2(69.2, 713.9), Vec2(69.7, 702.2), Vec2(72.7, 695.9), Vec2(75.7, 689.5), Vec2(100.2, 643.9), Vec2(124.6, 598.3), Vec2(130.2, 591.4), Vec2(135.8, 584.4), Vec2(132.3, 572.4), Vec2(128.9, 560.5), Vec2(126.0, 546.0), Vec2(123.1, 531.5), Vec2(121.1, 515.0), Vec2(119.0, 498.5), Vec2(118.0, 484.0), Vec2(117.0, 469.5), Vec2(117.0, 463.3), Vec2(117.0, 457.0), Vec2(119.8, 457.0), Vec2(122.7, 457.0), Vec2(123.3, 475.8), Vec2(123.9, 494.5), Vec2(125.0, 505.0), Vec2(126.1, 515.5), Vec2(128.0, 529.3), Vec2(130.0, 543.2), Vec2(132.1, 553.3), Vec2(134.2, 563.5), Vec2(136.5, 571.5), Vec2(138.9, 579.5), Vec2(142.0, 588.1), Vec2(145.1, 596.7), Vec2(149.3, 604.1), Vec2(153.6, 611.5), Vec2(161.5, 621.5), Vec2(169.3, 631.5), Vec2(175.6, 641.0), Vec2(181.8, 650.5), Vec2(186.8, 660.5), Vec2(191.8, 670.5), Vec2(194.9, 679.4), Vec2(198.0, 688.3), Vec2(199.5, 696.8), Vec2(201.0, 705.3), Vec2(201.0, 713.2), Vec2(201.0, 721.1), Vec2(199.4, 728.8), Vec2(197.8, 736.5), Vec2(194.2, 742.5), Vec2(190.5, 748.5), Vec2(185.5, 753.6), Vec2(180.5, 758.6), Vec2(174.0, 761.8), Vec2(167.5, 765.0), Vec2(160.6, 765.9), Vec2(153.6, 766.9), Vec2(146.3, 766.0), Vec2(139.0, 765.0), Vec2(133.7, 762.8), Vec2(128.5, 760.6), Vec2(125.2, 757.1), Vec2(121.8, 753.6), Vec2(120.4, 750.8), Vec2(118.9, 748.0), Vec2(118.0, 743.2), Vec2(117.1, 738.4), Vec2(117.9, 734.8), Vec2(118.7, 731.3), Vec2(142.9, 691.6), Vec2(167.2, 652.0), Vec2(166.8, 651.1), Vec2(166.5, 650.3), Vec2(146.4, 629.6), Vec2(126.2, 608.9), Vec2(120.7, 623.2), Vec2(115.1, 637.5), Vec2(101.0, 673.5), Vec2(86.8, 709.5), Vec2(84.8, 714.9), Vec2(82.8, 720.4), Vec2(82.3, 729.0), Vec2(81.7, 737.7), Vec2(83.4, 745.3), Vec2(85.1, 753.0), Vec2(88.8, 760.2), Vec2(92.4, 767.5), Vec2(100.4, 775.6), Vec2(108.5, 783.7), Vec2(115.0, 786.7), Vec2(121.5, 789.7), Vec2(125.5, 791.5), Vec2(129.5, 793.3), Vec2(136.5, 795.3), Vec2(143.5, 797.2), Vec2(155.2, 797.7), Vec2(166.8, 798.2), Vec2(174.4, 796.5), Vec2(182.1, 794.8), Vec2(190.3, 790.7), Vec2(198.5, 786.6), Vec2(206.7, 778.5), Vec2(214.9, 770.3), Vec2(219.3, 763.4), Vec2(223.8, 756.5), Vec2(226.3, 750.5), Vec2(228.8, 744.5), Vec2(230.3, 737.5), Vec2(231.9, 730.5), Vec2(232.5, 722.5), Vec2(233.2, 714.5), Vec2(231.6, 704.0), Vec2(230.0, 693.5), Vec2(227.4, 684.1), Vec2(224.9, 674.8), Vec2(220.8, 664.1), Vec2(216.7, 653.5), Vec2(209.0, 638.5), Vec2(201.4, 623.5), Vec2(192.0, 608.1), Vec2(182.7, 592.6), Vec2(178.4, 583.6), Vec2(174.1, 574.5), Vec2(171.1, 565.5), Vec2(168.1, 556.5), Vec2(165.6, 546.7), Vec2(163.1, 536.8), Vec2(160.6, 521.7), Vec2(158.0, 506.5), Vec2(157.1, 496.5), Vec2(156.1, 486.5), Vec2(155.4, 471.7), Vec2(154.7, 456.8), Vec2(157.1, 457.2), Vec2(159.5, 457.5), Vec2(159.7, 465.0), Vec2(160.0, 472.5), Vec2(161.0, 487.5), Vec2(162.1, 502.5), Vec2(164.1, 516.5), Vec2(166.1, 530.5), Vec2(169.5, 544.7), Vec2(172.9, 558.8), Vec2(176.6, 568.7), Vec2(180.2, 578.5), Vec2(183.3, 584.5), Vec2(186.3, 590.5), Vec2(199.5, 612.5), Vec2(212.8, 634.5), Vec2(219.2, 647.5), Vec2(225.7, 660.5), Vec2(229.4, 670.0), Vec2(233.0, 679.5), Vec2(235.6, 689.5), Vec2(238.2, 699.5), Vec2(238.7, 712.7), Vec2(239.3, 726.0), Vec2(238.1, 732.2), Vec2(236.9, 738.5), Vec2(235.3, 743.5), Vec2(233.7, 748.5), Vec2(230.3, 755.5), Vec2(226.8, 762.5), Vec2(221.7, 769.4), Vec2(216.5, 776.2), Vec2(207.5, 783.6), Vec2(198.5, 791.0), Vec2(191.8, 794.5), Vec2(185.1, 798.1), Vec2(177.4, 800.0), Vec2(169.7, 802.0), Vec2(158.6, 801.9), Vec2(147.5, 801.8), Vec2(141.0, 800.4)];
        PL.addShape(table, x, y, vs, 2);

        // right rail straight
        vs = [Vec2(339.4, 698.0), Vec2(333.2, 697.1), Vec2(328.0, 695.6), Vec2(322.7, 694.0), Vec2(318.4, 691.5), Vec2(314.1, 689.0), Vec2(310.6, 684.7), Vec2(307.1, 680.4), Vec2(304.0, 673.9), Vec2(301.0, 667.3), Vec2(298.5, 657.7), Vec2(296.1, 648.1), Vec2(294.5, 636.8), Vec2(292.9, 625.5), Vec2(291.9, 604.5), Vec2(290.9, 583.5), Vec2(292.0, 555.0), Vec2(293.1, 526.5), Vec2(294.6, 507.5), Vec2(296.1, 488.5), Vec2(297.5, 474.1), Vec2(299.0, 459.7), Vec2(300.1, 458.3), Vec2(301.2, 457.0), Vec2(302.9, 457.0), Vec2(304.6, 457.0), Vec2(303.3, 470.3), Vec2(302.0, 483.5), Vec2(300.4, 502.0), Vec2(298.8, 520.5), Vec2(298.3, 558.1), Vec2(297.7, 595.7), Vec2(298.9, 609.1), Vec2(300.1, 622.5), Vec2(301.6, 632.0), Vec2(303.2, 641.5), Vec2(305.6, 650.0), Vec2(308.0, 658.5), Vec2(311.7, 665.8), Vec2(315.3, 673.1), Vec2(320.7, 678.0), Vec2(326.1, 683.0), Vec2(332.8, 685.0), Vec2(339.4, 687.1), Vec2(346.5, 687.7), Vec2(353.7, 688.3), Vec2(360.6, 687.1), Vec2(367.5, 685.9), Vec2(373.6, 683.0), Vec2(379.6, 680.0), Vec2(383.9, 675.4), Vec2(388.1, 670.8), Vec2(390.5, 665.9), Vec2(393.0, 661.0), Vec2(394.6, 655.3), Vec2(396.1, 649.5), Vec2(396.7, 632.0), Vec2(397.3, 614.5), Vec2(396.1, 597.5), Vec2(394.9, 580.5), Vec2(392.9, 569.0), Vec2(391.0, 557.5), Vec2(388.3, 548.0), Vec2(385.7, 538.5), Vec2(384.8, 538.2), Vec2(384.0, 537.8), Vec2(384.0, 535.9), Vec2(384.0, 534.1), Vec2(381.7, 528.8), Vec2(379.3, 523.4), Vec2(376.6, 519.2), Vec2(373.9, 514.9), Vec2(372.6, 514.4), Vec2(371.3, 513.9), Vec2(355.4, 523.6), Vec2(339.5, 533.3), Vec2(338.7, 534.0), Vec2(337.9, 534.7), Vec2(335.9, 550.6), Vec2(333.9, 566.4), Vec2(334.6, 569.5), Vec2(335.2, 572.5), Vec2(342.5, 587.1), Vec2(349.8, 601.8), Vec2(353.4, 612.6), Vec2(357.0, 623.5), Vec2(358.5, 630.3), Vec2(360.1, 637.1), Vec2(358.9, 643.0), Vec2(357.8, 648.9), Vec2(355.9, 651.9), Vec2(354.0, 655.0), Vec2(351.6, 655.0), Vec2(349.1, 655.0), Vec2(346.8, 653.9), Vec2(344.5, 652.9), Vec2(342.1, 649.5), Vec2(339.8, 646.1), Vec2(337.4, 638.5), Vec2(335.0, 630.9), Vec2(333.5, 621.7), Vec2(332.1, 612.5), Vec2(331.0, 599.5), Vec2(329.9, 586.5), Vec2(329.8, 552.5), Vec2(329.7, 518.5), Vec2(331.4, 501.5), Vec2(333.1, 484.5), Vec2(335.1, 471.2), Vec2(337.0, 457.9), Vec2(339.0, 457.0), Vec2(341.1, 457.0), Vec2(341.6, 458.4), Vec2(342.2, 459.9), Vec2(340.1, 482.2), Vec2(338.1, 504.5), Vec2(337.5, 512.5), Vec2(336.9, 520.5), Vec2(336.4, 525.3), Vec2(335.8, 530.0), Vec2(355.2, 519.1), Vec2(373.0, 508.1), Vec2(376.6, 512.3), Vec2(380.2, 516.5), Vec2(384.0, 524.5), Vec2(387.8, 532.5), Vec2(390.9, 542.5), Vec2(393.9, 552.5), Vec2(395.9, 563.0), Vec2(397.9, 573.5), Vec2(399.2, 584.5), Vec2(400.5, 595.5), Vec2(400.4, 623.5), Vec2(400.3, 651.5), Vec2(398.2, 659.2), Vec2(396.1, 666.9), Vec2(393.3, 672.6), Vec2(390.5, 678.4), Vec2(385.8, 683.5), Vec2(381.1, 688.7), Vec2(375.3, 691.7), Vec2(369.5, 694.8), Vec2(364.0, 696.3), Vec2(358.5, 697.9), Vec2(352.0, 698.3), Vec2(345.5, 698.8), Vec2(339.4, 698.0)];
        PL.addShape(table, x, y, vs, 2);
    },
    addOutfield2(table, x, y) {
        const Vec2 = planck.Vec2;
        // outer left
        var vs = [Vec2(0, 677.5), Vec2(0, 513.0), Vec2(14.5, 513.0), Vec2(28.9, 513.0), Vec2(29.3, 639.3), Vec2(29.6, 765.5), Vec2(31.4, 772.5), Vec2(33.2, 779.5), Vec2(37.1, 787.7), Vec2(41.0, 795.9), Vec2(46.4, 801.7), Vec2(51.8, 807.5), Vec2(57.5, 810.9), Vec2(63.2, 814.4), Vec2(71.0, 816.6), Vec2(78.8, 818.9), Vec2(87.2, 819.6), Vec2(95.6, 820.2), Vec2(106.5, 819.0), Vec2(117.5, 817.8), Vec2(128.5, 815.0), Vec2(139.5, 812.2), Vec2(149.0, 808.6), Vec2(158.5, 804.9), Vec2(164.7, 802.4), Vec2(170.8, 800.0), Vec2(183.5, 795.1), Vec2(194.5, 790.3), Vec2(200.9, 787.1), Vec2(207.2, 783.9), Vec2(208.7, 784.4), Vec2(210.1, 785.0), Vec2(209.8, 787.7), Vec2(209.5, 790.4), Vec2(166.5, 816.2), Vec2(123.5, 842.0), Vec2(61.8, 842.0), Vec2(0, 842.0), Vec2(0, 677.5)];
        PL.addShape(table, x, y, vs, 1);
        // outer right
        vs = [Vec2(293.5, 816.2), Vec2(250.5, 790.4), Vec2(250.2, 787.2), Vec2(249.9, 784.0), Vec2(251.4, 783.4), Vec2(252.9, 782.8), Vec2(285.7, 801.1), Vec2(318.5, 819.3), Vec2(332.0, 827.1), Vec2(345.5, 834.8), Vec2(350.8, 836.5), Vec2(356.1, 838.3), Vec2(363.8, 837.8), Vec2(371.5, 837.3), Vec2(379.5, 834.2), Vec2(387.5, 831.0), Vec2(394.1, 825.1), Vec2(400.7, 819.2), Vec2(406.2, 810.8), Vec2(411.8, 802.5), Vec2(414.9, 796.2), Vec2(418.1, 789.9), Vec2(423.9, 773.7), Vec2(429.8, 757.5), Vec2(432.4, 745.5), Vec2(435.1, 733.5), Vec2(436.7, 726.0), Vec2(438.3, 718.5), Vec2(440.2, 710.0), Vec2(442.1, 701.5), Vec2(447.1, 677.5), Vec2(452.2, 653.5), Vec2(451.7, 636.5), Vec2(451.2, 619.5), Vec2(448.1, 604.5), Vec2(445.0, 589.5), Vec2(433.1, 553.0), Vec2(421.2, 516.5), Vec2(420.5, 514.8), Vec2(419.8, 513.0), Vec2(439.9, 513.0), Vec2(460.0, 513.0), Vec2(460.0, 677.5), Vec2(460.0, 842.0), Vec2(398.3, 842.0), Vec2(336.5, 842.0), Vec2(293.5, 816.2)];
        PL.addShape(table, x, y, vs, 1);
        // left island 1
        vs = [Vec2(164.5, 635.9), Vec2(146.5, 633.9), Vec2(145.6, 633.0), Vec2(144.7, 632.1), Vec2(145.8, 626.3), Vec2(147.0, 620.5), Vec2(151.2, 612.2), Vec2(155.5, 603.8), Vec2(158.8, 599.9), Vec2(162.2, 596.0), Vec2(164.1, 596.0), Vec2(166.1, 596.0), Vec2(166.4, 597.3), Vec2(166.7, 598.5), Vec2(168.0, 602.5), Vec2(169.3, 606.5), Vec2(171.1, 609.2), Vec2(173.0, 611.8), Vec2(184.0, 617.5), Vec2(195.0, 623.1), Vec2(195.0, 624.8), Vec2(194.9, 626.5), Vec2(192.2, 632.0), Vec2(189.5, 637.5), Vec2(186.0, 637.7), Vec2(182.5, 637.8), Vec2(164.5, 635.9)];
        PL.addShape(table, x, y, vs, 1);
        // left island 2
        vs = [Vec2(139.3, 769.3), Vec2(138.6, 767.6), Vec2(139.7, 767.3), Vec2(140.8, 766.9), Vec2(149.7, 761.0), Vec2(158.5, 755.2), Vec2(164.8, 749.5), Vec2(171.0, 743.8), Vec2(175.4, 738.1), Vec2(179.7, 732.5), Vec2(182.8, 727.0), Vec2(185.8, 721.5), Vec2(187.9, 714.5), Vec2(190.0, 707.5), Vec2(191.4, 697.2), Vec2(192.7, 686.9), Vec2(193.8, 685.8), Vec2(194.8, 684.8), Vec2(195.9, 685.4), Vec2(197.0, 686.1), Vec2(197.0, 690.9), Vec2(197.0, 695.7), Vec2(198.4, 699.6), Vec2(199.7, 703.5), Vec2(203.0, 710.4), Vec2(206.3, 717.3), Vec2(214.6, 726.1), Vec2(223.0, 734.9), Vec2(223.0, 736.5), Vec2(223.0, 738.1), Vec2(218.3, 740.1), Vec2(213.5, 742.2), Vec2(197.9, 747.5), Vec2(182.4, 752.9), Vec2(170.9, 757.6), Vec2(159.5, 762.2), Vec2(150.1, 766.6), Vec2(140.7, 771.0), Vec2(139.3, 769.3)];
        PL.addShape(table, x, y, vs, 1);
        // right island
        vs = [Vec2(357.4, 803.7), Vec2(352.3, 801.5), Vec2(335.4, 790.0), Vec2(318.5, 778.6), Vec2(306.0, 770.7), Vec2(293.5, 762.9), Vec2(279.1, 750.2), Vec2(264.7, 737.5), Vec2(258.9, 730.6), Vec2(253.0, 723.6), Vec2(253.0, 720.3), Vec2(253.0, 716.9), Vec2(254.4, 713.7), Vec2(255.7, 710.4), Vec2(292.6, 700.6), Vec2(329.5, 690.7), Vec2(331.3, 690.2), Vec2(333.1, 689.7), Vec2(333.7, 692.0), Vec2(334.3, 694.3), Vec2(339.1, 704.1), Vec2(343.9, 714.0), Vec2(348.0, 718.4), Vec2(352.1, 722.8), Vec2(358.5, 724.5), Vec2(364.9, 726.3), Vec2(374.2, 725.7), Vec2(383.5, 725.2), Vec2(392.7, 723.0), Vec2(401.9, 720.7), Vec2(403.5, 722.0), Vec2(405.0, 723.2), Vec2(405.0, 726.4), Vec2(405.0, 729.5), Vec2(402.1, 741.6), Vec2(399.3, 753.6), Vec2(395.2, 764.6), Vec2(391.2, 775.5), Vec2(388.0, 781.0), Vec2(384.8, 786.5), Vec2(381.3, 790.8), Vec2(377.8, 795.1), Vec2(371.0, 800.5), Vec2(364.3, 806.0), Vec2(363.4, 805.9), Vec2(362.5, 805.8), Vec2(357.4, 803.7)];
        PL.addShape(table, x, y, vs, 1);
        // left ramp base
        vs = [Vec2(92.5, 774.6), Vec2(88.5, 773.3), Vec2(84.2, 771.1), Vec2(79.9, 768.9), Vec2(76.1, 765.1), Vec2(72.2, 761.2), Vec2(69.4, 756.5), Vec2(66.7, 751.7), Vec2(64.3, 745.3), Vec2(62.0, 738.9), Vec2(60.5, 729.7), Vec2(59.0, 720.5), Vec2(59.0, 616.8), Vec2(59.0, 513.0), Vec2(64.0, 513.0), Vec2(69.0, 513.0), Vec2(69.0, 590.0), Vec2(69.0, 667.0), Vec2(75.8, 667.0), Vec2(82.6, 667.0), Vec2(84.9, 659.8), Vec2(87.2, 652.5), Vec2(92.0, 640.5), Vec2(96.8, 628.5), Vec2(102.5, 616.0), Vec2(108.1, 603.5), Vec2(111.2, 598.0), Vec2(114.3, 592.5), Vec2(117.7, 587.3), Vec2(121.2, 582.0), Vec2(122.6, 582.0), Vec2(124.1, 582.0), Vec2(124.7, 583.5), Vec2(125.2, 585.1), Vec2(123.7, 586.8), Vec2(122.2, 588.5), Vec2(112.2, 608.0), Vec2(102.1, 627.5), Vec2(92.6, 652.6), Vec2(83.1, 677.7), Vec2(80.1, 690.1), Vec2(77.1, 702.5), Vec2(75.5, 713.0), Vec2(73.9, 723.5), Vec2(73.7, 734.0), Vec2(73.5, 744.5), Vec2(76.4, 750.0), Vec2(79.3, 755.5), Vec2(84.6, 761.6), Vec2(90.0, 767.7), Vec2(96.8, 770.2), Vec2(103.5, 772.7), Vec2(103.5, 774.1), Vec2(103.5, 775.5), Vec2(100.0, 775.7), Vec2(96.5, 775.9), Vec2(92.5, 774.6)];
        PL.addShape(table, x, y, vs, 1);
        // right ramp base
        vs = [Vec2(372.8, 694.3), Vec2(371.2, 692.5), Vec2(348.0, 616.2), Vec2(324.8, 539.9), Vec2(325.4, 538.5), Vec2(325.9, 537.0), Vec2(327.4, 537.0), Vec2(328.9, 537.0), Vec2(329.5, 538.0), Vec2(330.2, 539.1), Vec2(352.9, 613.8), Vec2(375.7, 688.5), Vec2(377.3, 688.8), Vec2(378.9, 689.1), Vec2(383.7, 686.9), Vec2(388.5, 684.7), Vec2(394.0, 681.0), Vec2(399.5, 677.2), Vec2(402.1, 674.4), Vec2(404.8, 671.5), Vec2(407.3, 667.8), Vec2(409.7, 664.1), Vec2(394.9, 600.3), Vec2(380.0, 536.5), Vec2(380.3, 534.5), Vec2(380.5, 532.5), Vec2(383.6, 532.2), Vec2(386.7, 531.9), Vec2(387.7, 533.2), Vec2(388.8, 534.5), Vec2(395.8, 557.0), Vec2(402.9, 579.5), Vec2(407.5, 595.0), Vec2(412.1, 610.5), Vec2(415.3, 623.5), Vec2(418.4, 636.5), Vec2(418.4, 648.5), Vec2(418.5, 660.5), Vec2(415.3, 667.3), Vec2(412.1, 674.0), Vec2(406.8, 679.3), Vec2(401.5, 684.5), Vec2(395.6, 688.4), Vec2(389.7, 692.3), Vec2(384.8, 694.2), Vec2(380.0, 696.0), Vec2(377.1, 696.0), Vec2(374.3, 696.0), Vec2(372.8, 694.3)];
        PL.addShape(table, x, y, vs, 1);

        PL.addPopBumper(table, 17.4, 72.8, 1);
        PL.addPopBumper(table, 16.7, 63.7 ,1);
        PL.addPopBumper(table, 9, 68.4, 1);

        // drop targets
        PL.addWall(table, 
            {x: 25.9, y: 70.9},
            {x: 27.8, y: 70.4}, 
            1, {group: "todo", id: 0}
        );
        PL.addWall(table, 
            {x: 28.3, y: 70.3},
            {x: 30.4, y: 69.7},
            1, {group: "todo", id: 1}
        );
        PL.addWall(table, 
            {x: 30.9, y: 69.7},
            {x: 33, y: 69.1},
            1, {group: "todo", id: 2}
        );

        // left ramp down
        PL.addRamp(table, {x: 7.8, y: 55.1}, {x: 11.1, y: 57}, 2, 1);
        // left ramp up
        PL.addRamp(table, {x: 8, y: 60.8}, {x: 8, y: 64}, 1, 2);
        // rigth ramp down
        PL.addRamp(table, {x: 34.3, y: 55.1}, {x: 37.4, y: 54}, 2, 1);
        // right ramp up
        PL.addRamp(table, {x: 37.9, y: 63.6}, {x: 39.1, y: 66.8}, 1, 2);

        PL.addOutfield2Rails(table, x, y);
    },
    addOutfield2Rails : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // left rail side 1
        var vs = [Vec2(41.0, 630.4), Vec2(34.5, 629.0), Vec2(26.4, 625.0), Vec2(18.2, 621.0), Vec2(12.8, 615.6), Vec2(7.4, 610.2), Vec2(4.8, 604.9), Vec2(2.2, 599.5), Vec2(1.1, 594.2), Vec2(0, 588.9), Vec2(0, 486.2), Vec2(0, 383.5), Vec2(2.0, 386.0), Vec2(4.0, 388.5), Vec2(4.0, 489.0), Vec2(4.0, 589.6), Vec2(5.5, 595.5), Vec2(7.1, 601.5), Vec2(10.5, 606.7), Vec2(14.0, 611.9), Vec2(19.0, 615.9), Vec2(24.0, 619.8), Vec2(29.6, 622.5), Vec2(35.2, 625.1), Vec2(42.3, 626.7), Vec2(49.5, 628.2), Vec2(59.0, 627.5), Vec2(68.5, 626.9), Vec2(73.0, 625.8), Vec2(77.5, 624.7), Vec2(83.5, 621.9), Vec2(89.5, 619.1), Vec2(95.0, 614.4), Vec2(100.5, 609.7), Vec2(106.1, 603.4), Vec2(111.7, 597.0), Vec2(112.3, 587.3), Vec2(112.8, 577.5), Vec2(115.4, 568.7), Vec2(118.0, 559.9), Vec2(120.9, 553.7), Vec2(123.9, 547.5), Vec2(126.2, 543.5), Vec2(128.5, 539.6), Vec2(133.7, 534.0), Vec2(139.0, 528.5), Vec2(139.0, 505.8), Vec2(139.0, 483.0), Vec2(102.5, 483.0), Vec2(66.0, 483.0), Vec2(66.0, 530.8), Vec2(66.0, 578.5), Vec2(64.0, 582.8), Vec2(62.1, 587.1), Vec2(57.5, 589.1), Vec2(52.9, 591.2), Vec2(49.0, 590.5), Vec2(45.0, 589.8), Vec2(41.8, 587.6), Vec2(38.5, 585.5), Vec2(36.8, 582.0), Vec2(35.0, 578.5), Vec2(35.0, 476.2), Vec2(35.0, 373.8), Vec2(17.5, 355.7), Vec2(0, 337.5), Vec2(0.1, 334.5), Vec2(0.1, 331.5), Vec2(19.5, 351.5), Vec2(38.9, 371.5), Vec2(39.0, 475.4), Vec2(39.0, 579.2), Vec2(42.4, 582.6), Vec2(45.8, 586.0), Vec2(50.5, 586.0), Vec2(55.1, 586.0), Vec2(57.2, 584.9), Vec2(59.2, 583.8), Vec2(60.6, 580.5), Vec2(62.0, 577.1), Vec2(62.0, 528.0), Vec2(62.0, 479.0), Vec2(102.3, 479.2), Vec2(142.5, 479.5), Vec2(142.8, 505.0), Vec2(143.0, 530.4), Vec2(138.1, 535.5), Vec2(133.2, 540.5), Vec2(129.1, 546.9), Vec2(125.1, 553.2), Vec2(122.1, 560.9), Vec2(119.2, 568.5), Vec2(117.6, 576.4), Vec2(116.0, 584.2), Vec2(116.0, 591.5), Vec2(116.0, 598.7), Vec2(107.8, 607.5), Vec2(99.6, 616.2), Vec2(92.7, 621.1), Vec2(85.8, 625.9), Vec2(79.7, 627.9), Vec2(73.5, 630.0), Vec2(66.0, 631.0), Vec2(58.5, 631.9), Vec2(53.0, 631.9), Vec2(47.5, 631.8), Vec2(41.0, 630.4)];
        //PL.addShape(table, x, y, vs, 2);
        // right rail side 1
        vs = [Vec2(402.7, 716.0), Vec2(399.0, 715.0), Vec2(394.7, 712.9), Vec2(390.5, 710.8), Vec2(386.3, 707.4), Vec2(382.2, 704.0), Vec2(376.5, 697.6), Vec2(370.9, 691.1), Vec2(341.9, 621.3), Vec2(313.0, 551.5), Vec2(313.0, 550.0), Vec2(313.0, 548.4), Vec2(346.3, 535.7), Vec2(379.5, 523.0), Vec2(380.7, 523.0), Vec2(381.9, 523.0), Vec2(382.5, 523.9), Vec2(383.0, 524.9), Vec2(398.6, 594.2), Vec2(414.1, 663.5), Vec2(414.7, 666.3), Vec2(415.3, 669.0), Vec2(416.9, 669.0), Vec2(418.6, 669.0), Vec2(419.8, 667.8), Vec2(421.0, 666.6), Vec2(421.0, 519.1), Vec2(421.1, 371.5), Vec2(440.5, 351.5), Vec2(459.9, 331.5), Vec2(459.9, 334.5), Vec2(460.0, 337.5), Vec2(442.5, 355.7), Vec2(425.0, 373.8), Vec2(425.0, 520.2), Vec2(425.0, 666.5), Vec2(423.0, 669.7), Vec2(421.1, 672.9), Vec2(418.5, 673.6), Vec2(415.9, 674.3), Vec2(413.5, 673.2), Vec2(411.1, 672.1), Vec2(409.6, 666.8), Vec2(408.1, 661.5), Vec2(393.5, 596.0), Vec2(378.9, 530.5), Vec2(349.0, 540.4), Vec2(319.5, 551.7), Vec2(319.8, 553.1), Vec2(320.0, 554.5), Vec2(348.4, 623.0), Vec2(376.7, 691.5), Vec2(383.5, 698.7), Vec2(390.3, 706.0), Vec2(396.9, 709.2), Vec2(403.5, 712.5), Vec2(410.5, 712.5), Vec2(417.5, 712.5), Vec2(424.0, 709.4), Vec2(430.5, 706.3), Vec2(438.0, 700.2), Vec2(445.5, 694.1), Vec2(448.3, 690.5), Vec2(451.1, 686.9), Vec2(453.3, 682.2), Vec2(455.5, 677.5), Vec2(455.8, 533.0), Vec2(456.1, 388.5), Vec2(458.0, 386.0), Vec2(460.0, 383.5), Vec2(460.0, 530.6), Vec2(460.0, 677.6), Vec2(456.9, 684.0), Vec2(453.9, 690.3), Vec2(448.2, 696.3), Vec2(442.5, 702.2), Vec2(436.4, 706.6), Vec2(430.3, 711.0), Vec2(424.7, 713.5), Vec2(419.2, 716.1), Vec2(412.8, 716.5), Vec2(406.5, 716.9), Vec2(402.7, 716.0)];
        //PL.addShape(table, x, y, vs, 2);

        // left rail straight
        vs = [Vec2(86.4, 689.7), Vec2(83.3, 688.5), Vec2(79.8, 686.1), Vec2(76.3, 683.6), Vec2(73.1, 679.0), Vec2(69.9, 674.3), Vec2(67.7, 667.9), Vec2(65.5, 661.5), Vec2(65.2, 597.3), Vec2(64.9, 533.0), Vec2(95.5, 533.0), Vec2(126.0, 533.0), Vec2(125.6, 530.8), Vec2(125.1, 528.5), Vec2(123.5, 518.0), Vec2(121.9, 507.5), Vec2(120.5, 493.0), Vec2(119.0, 478.5), Vec2(119.0, 467.8), Vec2(119.0, 457.0), Vec2(121.0, 457.0), Vec2(123.0, 457.0), Vec2(123.0, 467.3), Vec2(123.0, 477.6), Vec2(124.6, 494.1), Vec2(126.1, 510.5), Vec2(127.1, 510.8), Vec2(128.0, 511.2), Vec2(128.0, 516.3), Vec2(128.0, 521.5), Vec2(133.5, 553.4), Vec2(139.0, 585.2), Vec2(139.0, 595.2), Vec2(139.0, 605.1), Vec2(137.4, 612.8), Vec2(135.8, 620.4), Vec2(132.3, 629.5), Vec2(128.9, 638.5), Vec2(128.2, 640.5), Vec2(127.6, 642.5), Vec2(126.6, 643.6), Vec2(125.5, 644.6), Vec2(123.1, 648.1), Vec2(120.8, 651.5), Vec2(118.1, 653.5), Vec2(115.5, 655.5), Vec2(111.2, 655.5), Vec2(106.9, 655.5), Vec2(102.8, 651.2), Vec2(98.8, 646.9), Vec2(99.2, 634.2), Vec2(99.5, 621.5), Vec2(106.2, 608.0), Vec2(112.9, 594.5), Vec2(116.4, 589.2), Vec2(120.0, 583.8), Vec2(120.0, 560.4), Vec2(120.0, 537.0), Vec2(94.5, 537.0), Vec2(68.9, 537.0), Vec2(69.2, 598.8), Vec2(69.5, 660.5), Vec2(71.8, 667.2), Vec2(74.1, 673.9), Vec2(77.3, 678.1), Vec2(80.5, 682.3), Vec2(84.5, 684.4), Vec2(88.5, 686.5), Vec2(100.8, 686.8), Vec2(113.1, 687.2), Vec2(121.3, 685.5), Vec2(129.5, 683.9), Vec2(136.8, 680.5), Vec2(144.2, 677.0), Vec2(150.4, 670.7), Vec2(156.7, 664.5), Vec2(160.8, 656.0), Vec2(164.9, 647.5), Vec2(166.9, 639.0), Vec2(169.0, 630.5), Vec2(170.1, 620.0), Vec2(171.3, 609.5), Vec2(170.7, 593.0), Vec2(170.0, 576.5), Vec2(166.5, 557.0), Vec2(162.9, 537.5), Vec2(160.5, 520.5), Vec2(158.1, 503.5), Vec2(157.1, 490.0), Vec2(156.1, 476.5), Vec2(155.5, 466.8), Vec2(154.8, 457.0), Vec2(156.8, 457.0), Vec2(158.8, 457.0), Vec2(159.4, 465.8), Vec2(160.0, 474.5), Vec2(161.6, 493.4), Vec2(163.1, 512.4), Vec2(165.6, 527.9), Vec2(168.0, 543.5), Vec2(171.2, 560.5), Vec2(174.4, 577.5), Vec2(174.4, 600.0), Vec2(174.5, 622.5), Vec2(172.8, 631.5), Vec2(171.1, 640.5), Vec2(168.6, 647.9), Vec2(166.0, 655.2), Vec2(162.4, 661.7), Vec2(158.8, 668.2), Vec2(153.1, 673.9), Vec2(147.5, 679.5), Vec2(140.5, 683.2), Vec2(133.5, 686.8), Vec2(127.5, 688.5), Vec2(121.5, 690.2), Vec2(105.5, 690.6), Vec2(89.5, 691.0), Vec2(86.4, 689.7)];
        PL.addShape(table, x, y, vs, 2);
        // right rail straight
        vs = [Vec2(357.3, 731.9), Vec2(352.1, 731.0), Vec2(345.8, 728.6), Vec2(339.5, 726.3), Vec2(334.0, 723.0), Vec2(328.5, 719.8), Vec2(322.3, 714.9), Vec2(316.1, 709.9), Vec2(308.9, 702.2), Vec2(301.7, 694.5), Vec2(296.8, 687.5), Vec2(291.8, 680.5), Vec2(287.7, 673.5), Vec2(283.7, 666.6), Vec2(276.7, 651.5), Vec2(269.8, 636.5), Vec2(265.9, 625.0), Vec2(262.1, 613.5), Vec2(260.5, 604.6), Vec2(259.0, 595.7), Vec2(259.0, 587.0), Vec2(259.0, 578.3), Vec2(260.5, 569.9), Vec2(261.9, 561.5), Vec2(264.3, 554.5), Vec2(266.7, 547.5), Vec2(269.7, 540.9), Vec2(272.7, 534.3), Vec2(280.1, 522.9), Vec2(287.4, 511.5), Vec2(290.7, 503.6), Vec2(294.0, 495.8), Vec2(296.1, 488.1), Vec2(298.1, 480.5), Vec2(299.3, 468.8), Vec2(300.4, 457.0), Vec2(302.7, 457.0), Vec2(305.0, 457.0), Vec2(305.0, 458.8), Vec2(305.0, 460.5), Vec2(304.0, 469.5), Vec2(302.9, 478.5), Vec2(300.5, 487.9), Vec2(298.0, 497.2), Vec2(294.5, 505.3), Vec2(291.0, 513.3), Vec2(284.1, 523.9), Vec2(277.2, 534.5), Vec2(275.1, 538.7), Vec2(273.0, 542.8), Vec2(270.2, 550.2), Vec2(267.3, 557.5), Vec2(265.4, 565.0), Vec2(263.5, 572.5), Vec2(263.5, 587.0), Vec2(263.6, 601.5), Vec2(266.4, 612.3), Vec2(269.2, 623.2), Vec2(273.5, 633.8), Vec2(277.8, 644.5), Vec2(282.4, 654.5), Vec2(287.1, 664.5), Vec2(291.6, 672.1), Vec2(296.0, 679.7), Vec2(301.5, 687.1), Vec2(307.0, 694.5), Vec2(315.2, 702.8), Vec2(323.5, 711.2), Vec2(332.4, 717.0), Vec2(341.2, 722.9), Vec2(348.9, 725.4), Vec2(356.5, 727.9), Vec2(362.5, 728.5), Vec2(368.5, 729.2), Vec2(375.6, 727.7), Vec2(382.8, 726.2), Vec2(388.0, 723.5), Vec2(393.2, 720.9), Vec2(397.7, 716.7), Vec2(402.1, 712.6), Vec2(404.5, 708.5), Vec2(406.9, 704.4), Vec2(408.5, 700.0), Vec2(410.1, 695.5), Vec2(411.0, 688.5), Vec2(411.9, 681.5), Vec2(411.0, 675.5), Vec2(410.2, 669.5), Vec2(395.1, 602.5), Vec2(380.0, 535.5), Vec2(379.3, 532.3), Vec2(378.5, 529.1), Vec2(355.8, 537.8), Vec2(333.0, 546.5), Vec2(333.0, 547.6), Vec2(333.0, 548.7), Vec2(354.0, 617.1), Vec2(375.0, 685.5), Vec2(375.0, 688.3), Vec2(375.0, 691.2), Vec2(372.1, 694.1), Vec2(369.2, 697.0), Vec2(367.1, 697.0), Vec2(365.0, 697.0), Vec2(358.0, 695.1), Vec2(351.0, 693.1), Vec2(345.9, 690.3), Vec2(340.8, 687.6), Vec2(335.1, 682.0), Vec2(329.4, 676.5), Vec2(324.7, 669.9), Vec2(320.0, 663.2), Vec2(312.2, 647.9), Vec2(304.4, 632.5), Vec2(300.7, 622.2), Vec2(297.0, 611.9), Vec2(294.9, 601.7), Vec2(292.8, 591.5), Vec2(293.3, 581.0), Vec2(293.7, 570.5), Vec2(296.0, 564.7), Vec2(298.2, 558.8), Vec2(302.6, 553.7), Vec2(307.0, 548.5), Vec2(312.6, 540.2), Vec2(318.1, 532.0), Vec2(322.2, 523.3), Vec2(326.2, 514.6), Vec2(329.2, 505.7), Vec2(332.1, 496.8), Vec2(334.1, 488.2), Vec2(336.0, 479.5), Vec2(336.7, 468.3), Vec2(337.3, 457.0), Vec2(340.2, 457.0), Vec2(343.0, 457.0), Vec2(343.0, 464.4), Vec2(343.0, 471.7), Vec2(341.5, 480.6), Vec2(339.9, 489.5), Vec2(336.4, 501.9), Vec2(332.8, 514.2), Vec2(330.3, 520.4), Vec2(327.8, 526.5), Vec2(325.5, 531.0), Vec2(323.2, 535.5), Vec2(318.7, 541.7), Vec2(314.1, 547.9), Vec2(317.8, 546.5), Vec2(321.5, 545.2), Vec2(350.5, 534.1), Vec2(379.5, 523.0), Vec2(381.2, 523.0), Vec2(382.9, 523.0), Vec2(383.4, 524.8), Vec2(383.9, 526.5), Vec2(388.9, 550.0), Vec2(393.9, 573.5), Vec2(403.8, 618.0), Vec2(413.8, 662.5), Vec2(414.8, 672.1), Vec2(415.8, 681.7), Vec2(414.9, 689.1), Vec2(414.0, 696.5), Vec2(412.9, 700.0), Vec2(411.7, 703.5), Vec2(409.3, 708.4), Vec2(406.8, 713.3), Vec2(401.6, 718.5), Vec2(396.4, 723.7), Vec2(388.9, 727.3), Vec2(381.5, 730.8), Vec2(375.6, 731.9), Vec2(369.6, 733.0), Vec2(366.1, 732.9), Vec2(362.5, 732.8), Vec2(357.3, 731.9)];
        PL.addShape(table, x, y, vs, 2);
    },
    addOutfield3 : function(table, x, y) {
        const Vec2 = planck.Vec2;
        // left outer
        var vs = [Vec2(0, 677.5), Vec2(0, 513.0), Vec2(14.5, 513.0), Vec2(29.0, 513.0), Vec2(29.0, 548.8), Vec2(29.0, 584.5), Vec2(27.0, 595.0), Vec2(25.1, 605.5), Vec2(23.6, 617.5), Vec2(22.1, 629.5), Vec2(20.8, 642.5), Vec2(19.5, 655.5), Vec2(19.5, 691.5), Vec2(19.5, 727.5), Vec2(22.7, 739.3), Vec2(25.9, 751.1), Vec2(30.9, 761.6), Vec2(36.0, 772.2), Vec2(40.4, 778.8), Vec2(44.8, 785.5), Vec2(51.4, 793.2), Vec2(57.9, 800.8), Vec2(66.2, 807.8), Vec2(74.5, 814.8), Vec2(81.5, 819.2), Vec2(88.5, 823.7), Vec2(95.8, 827.4), Vec2(103.1, 831.1), Vec2(112.3, 834.1), Vec2(121.5, 837.2), Vec2(122.8, 838.2), Vec2(124.0, 839.2), Vec2(124.0, 840.6), Vec2(124.0, 842.0), Vec2(62.0, 842.0), Vec2(0, 842.0), Vec2(0, 677.5)];
        PL.addShape(table, x, y, vs, 1);
        // right outer
        vs = [Vec2(300.8, 826.5), Vec2(266.0, 811.1), Vec2(253.8, 809.6), Vec2(241.5, 808.1), Vec2(228.5, 807.0), Vec2(215.5, 805.9), Vec2(213.2, 805.4), Vec2(210.9, 804.9), Vec2(211.2, 801.7), Vec2(211.5, 798.5), Vec2(230.5, 797.8), Vec2(249.5, 797.2), Vec2(266.5, 795.6), Vec2(283.4, 793.9), Vec2(295.9, 791.9), Vec2(308.3, 789.9), Vec2(319.9, 787.4), Vec2(331.5, 784.8), Vec2(346.5, 779.9), Vec2(361.5, 774.9), Vec2(375.5, 768.0), Vec2(389.5, 761.0), Vec2(398.5, 754.5), Vec2(407.5, 748.1), Vec2(415.7, 740.0), Vec2(424.0, 732.0), Vec2(429.9, 724.0), Vec2(435.8, 716.1), Vec2(439.9, 708.6), Vec2(444.1, 701.2), Vec2(447.5, 691.1), Vec2(451.0, 681.1), Vec2(453.0, 669.8), Vec2(455.0, 658.5), Vec2(455.1, 642.5), Vec2(455.1, 626.5), Vec2(454.0, 617.7), Vec2(452.9, 608.9), Vec2(450.9, 599.2), Vec2(449.0, 589.5), Vec2(445.9, 579.5), Vec2(442.9, 569.5), Vec2(441.0, 564.5), Vec2(439.2, 559.5), Vec2(434.2, 548.0), Vec2(429.2, 536.5), Vec2(425.0, 526.0), Vec2(420.8, 515.5), Vec2(420.3, 514.3), Vec2(419.8, 513.0), Vec2(439.9, 513.0), Vec2(460.0, 513.0), Vec2(460.0, 677.5), Vec2(460.0, 842.0), Vec2(397.8, 841.9), Vec2(335.5, 841.8), Vec2(300.8, 826.5)];
        PL.addShape(table, x, y, vs, 1);
        // left island 1
        vs = [Vec2(59.7, 544.3), Vec2(59.0, 543.7), Vec2(59.0, 528.3), Vec2(59.0, 513.0), Vec2(64.0, 513.0), Vec2(69.0, 513.0), Vec2(69.0, 514.1), Vec2(69.0, 515.1), Vec2(65.4, 529.3), Vec2(61.9, 543.5), Vec2(61.1, 544.3), Vec2(60.4, 545.0), Vec2(59.7, 544.3)];
        PL.addShape(table, x, y, vs, 1);
        // left island 2
        vs = [Vec2(153.0, 737.6), Vec2(143.5, 734.4), Vec2(138.0, 731.4), Vec2(132.6, 728.5), Vec2(129.0, 725.0), Vec2(125.4, 721.4), Vec2(120.0, 715.0), Vec2(114.7, 708.5), Vec2(112.1, 703.0), Vec2(109.5, 697.5), Vec2(109.5, 687.0), Vec2(109.5, 676.5), Vec2(112.7, 669.7), Vec2(115.9, 662.9), Vec2(121.7, 656.8), Vec2(127.5, 650.7), Vec2(136.2, 643.4), Vec2(144.9, 636.1), Vec2(150.9, 632.1), Vec2(156.9, 628.0), Vec2(163.7, 624.5), Vec2(170.5, 621.0), Vec2(175.8, 619.5), Vec2(181.1, 617.9), Vec2(182.0, 618.5), Vec2(183.0, 619.1), Vec2(183.0, 620.4), Vec2(183.0, 621.7), Vec2(181.1, 623.4), Vec2(179.1, 625.2), Vec2(170.6, 642.4), Vec2(162.1, 659.5), Vec2(158.7, 669.5), Vec2(155.2, 679.5), Vec2(153.4, 687.5), Vec2(151.6, 695.5), Vec2(151.6, 706.0), Vec2(151.6, 716.5), Vec2(154.0, 722.5), Vec2(156.3, 728.4), Vec2(161.2, 733.2), Vec2(166.2, 738.1), Vec2(165.6, 739.5), Vec2(165.1, 741.0), Vec2(163.8, 740.9), Vec2(162.5, 740.9), Vec2(153.0, 737.6)];
        PL.addShape(table, x, y, vs, 1);
        // right island 1
        vs = [Vec2(290.0, 619.5), Vec2(272.5, 614.1), Vec2(270.7, 614.0), Vec2(268.9, 614.0), Vec2(269.5, 611.8), Vec2(270.2, 609.5), Vec2(272.2, 605.0), Vec2(274.2, 600.5), Vec2(276.1, 595.1), Vec2(278.0, 589.7), Vec2(278.0, 584.1), Vec2(278.0, 578.5), Vec2(275.8, 571.2), Vec2(273.7, 563.9), Vec2(275.3, 562.3), Vec2(276.8, 560.7), Vec2(278.3, 561.3), Vec2(279.7, 561.8), Vec2(295.9, 590.4), Vec2(312.1, 619.0), Vec2(311.8, 621.8), Vec2(311.5, 624.5), Vec2(309.5, 624.7), Vec2(307.5, 624.9), Vec2(290.0, 619.5)];
        PL.addShape(table, x, y, vs, 1);
        // right island 2
        vs = [Vec2(227.8, 748.9), Vec2(219.1, 748.2), Vec2(216.0, 747.5), Vec2(213.0, 746.8), Vec2(215.1, 743.9), Vec2(217.2, 742.1), Vec2(220.9, 740.9), Vec2(224.5, 739.7), Vec2(233.5, 735.2), Vec2(242.5, 730.8), Vec2(248.6, 726.5), Vec2(254.6, 722.2), Vec2(258.4, 718.0), Vec2(262.2, 713.8), Vec2(264.1, 709.6), Vec2(266.0, 705.5), Vec2(266.0, 702.7), Vec2(266.0, 699.9), Vec2(267.4, 699.4), Vec2(268.9, 698.8), Vec2(283.2, 704.3), Vec2(297.5, 709.9), Vec2(300.3, 710.5), Vec2(303.0, 711.2), Vec2(303.4, 711.9), Vec2(303.9, 712.6), Vec2(306.6, 724.9), Vec2(309.3, 737.2), Vec2(307.9, 738.1), Vec2(306.5, 738.9), Vec2(297.0, 742.0), Vec2(287.5, 745.0), Vec2(280.5, 746.4), Vec2(273.5, 747.9), Vec2(262.0, 748.9), Vec2(250.5, 750.0), Vec2(243.5, 749.9), Vec2(236.5, 749.7), Vec2(227.8, 748.9)];
        PL.addShape(table, x, y, vs, 1);
        // left ramp base
        vs = [Vec2(119.8, 787.1), Vec2(112.0, 784.3), Vec2(105.8, 780.0), Vec2(99.5, 775.7), Vec2(94.3, 771.6), Vec2(89.2, 767.5), Vec2(80.5, 760.5), Vec2(71.8, 753.5), Vec2(65.8, 745.7), Vec2(59.8, 737.9), Vec2(57.2, 731.7), Vec2(54.7, 725.5), Vec2(53.3, 720.0), Vec2(51.8, 714.5), Vec2(51.3, 691.0), Vec2(50.8, 667.5), Vec2(51.9, 650.0), Vec2(53.1, 632.5), Vec2(54.6, 621.9), Vec2(56.0, 611.3), Vec2(57.6, 605.4), Vec2(59.1, 599.6), Vec2(60.9, 595.6), Vec2(62.7, 591.5), Vec2(70.5, 582.5), Vec2(78.3, 573.5), Vec2(90.9, 561.0), Vec2(103.5, 548.5), Vec2(115.0, 539.1), Vec2(126.5, 529.7), Vec2(136.0, 523.5), Vec2(145.5, 517.3), Vec2(148.8, 515.6), Vec2(152.2, 513.9), Vec2(153.6, 514.4), Vec2(155.0, 514.9), Vec2(155.0, 516.4), Vec2(155.0, 517.8), Vec2(147.8, 522.0), Vec2(140.5, 526.3), Vec2(134.0, 530.8), Vec2(127.5, 535.4), Vec2(117.0, 543.9), Vec2(106.5, 552.4), Vec2(91.9, 567.1), Vec2(77.2, 581.9), Vec2(75.1, 584.8), Vec2(73.0, 587.8), Vec2(84.1, 598.0), Vec2(95.1, 608.3), Vec2(104.8, 598.8), Vec2(114.5, 589.4), Vec2(120.5, 584.8), Vec2(126.5, 580.3), Vec2(136.5, 573.8), Vec2(146.5, 567.2), Vec2(160.7, 559.5), Vec2(175.0, 551.8), Vec2(176.6, 552.4), Vec2(178.1, 553.0), Vec2(177.8, 554.7), Vec2(177.5, 556.4), Vec2(172.0, 559.5), Vec2(166.5, 562.7), Vec2(156.0, 568.4), Vec2(145.5, 574.2), Vec2(140.9, 576.8), Vec2(136.3, 579.5), Vec2(132.4, 582.8), Vec2(128.5, 586.1), Vec2(122.2, 591.3), Vec2(115.8, 596.5), Vec2(106.2, 605.5), Vec2(96.5, 614.5), Vec2(91.0, 622.0), Vec2(85.5, 629.6), Vec2(82.3, 635.6), Vec2(79.1, 641.6), Vec2(76.1, 650.6), Vec2(73.1, 659.5), Vec2(72.0, 669.5), Vec2(70.8, 679.5), Vec2(71.5, 688.0), Vec2(72.2, 696.5), Vec2(73.6, 704.1), Vec2(75.0, 711.7), Vec2(77.5, 719.0), Vec2(80.0, 726.3), Vec2(84.0, 733.7), Vec2(88.0, 741.1), Vec2(92.4, 746.8), Vec2(96.8, 752.5), Vec2(108.7, 764.0), Vec2(120.5, 775.5), Vec2(126.8, 780.3), Vec2(133.1, 785.1), Vec2(132.8, 787.3), Vec2(132.5, 789.5), Vec2(130.0, 789.7), Vec2(127.5, 789.9), Vec2(119.8, 787.1)];
        PL.addShape(table, x, y, vs, 1);
        // right ramp base
        vs = [Vec2(397.3, 648.9), Vec2(394.2, 646.9), Vec2(392.6, 644.5), Vec2(391.0, 642.0), Vec2(377.2, 642.0), Vec2(363.4, 642.0), Vec2(362.1, 632.8), Vec2(360.9, 623.5), Vec2(359.4, 616.0), Vec2(357.9, 608.5), Vec2(354.0, 596.9), Vec2(350.2, 585.3), Vec2(346.0, 576.9), Vec2(341.8, 568.5), Vec2(339.9, 565.8), Vec2(338.0, 563.2), Vec2(338.0, 561.6), Vec2(338.0, 560.1), Vec2(340.0, 559.0), Vec2(342.0, 557.9), Vec2(343.0, 558.5), Vec2(343.9, 559.0), Vec2(350.4, 569.0), Vec2(356.9, 578.9), Vec2(365.1, 592.2), Vec2(373.3, 605.5), Vec2(385.8, 624.8), Vec2(398.2, 644.0), Vec2(400.5, 644.0), Vec2(402.8, 644.0), Vec2(409.6, 640.0), Vec2(416.5, 636.0), Vec2(418.2, 636.0), Vec2(419.9, 636.0), Vec2(421.0, 634.1), Vec2(422.0, 632.1), Vec2(422.0, 630.2), Vec2(422.0, 628.3), Vec2(408.9, 600.9), Vec2(395.8, 573.5), Vec2(390.3, 559.8), Vec2(384.8, 546.1), Vec2(385.2, 544.3), Vec2(385.5, 542.5), Vec2(387.6, 542.2), Vec2(389.7, 541.9), Vec2(390.7, 543.2), Vec2(391.8, 544.5), Vec2(408.4, 585.7), Vec2(425.0, 626.9), Vec2(425.0, 630.7), Vec2(425.0, 634.5), Vec2(423.2, 637.9), Vec2(421.3, 641.3), Vec2(412.9, 646.2), Vec2(404.5, 651.0), Vec2(402.5, 651.0), Vec2(400.5, 651.0), Vec2(397.3, 648.9)];
        PL.addShape(table, x, y, vs, 1);

        PL.addPopBumper(table, 30.7, 71.2, 1);
        PL.addPopBumper(table, 28.8, 62.4 ,1);
        PL.addPopBumper(table, 37.9, 65.6, 1);

        // left ramp up
        PL.addRamp(table, {x: 11.9, y: 56.5}, {x: 8.9, y: 59.2}, 1, 2);

        // left ramp down
        PL.addRamp(table, {x: 15.4, y: 52.6}, {x: 16.8, y: 54.8}, 2, 1);

        // right ramp up
        PL.addRamp(table, {x: 40.6, y: 63.3}, {x: 39, y: 59.9}, 1, 2);

        // right ramp down
        PL.addRamp(table, {x: 35.3, y: 56}, {x: 37.9, y: 55.1}, 2, 1);

        PL.addOutfield3Rails(table, x, y);
    },
    addOutfield3Rails(table, x, y) {
        const Vec2 = planck.Vec2;
        // left ramp side 1
        var vs = [Vec2(41.0, 630.4), Vec2(34.5, 629.0), Vec2(26.4, 625.0), Vec2(18.2, 621.0), Vec2(12.8, 615.6), Vec2(7.4, 610.2), Vec2(4.8, 604.9), Vec2(2.2, 599.5), Vec2(1.1, 594.2), Vec2(0, 588.9), Vec2(0, 486.2), Vec2(0, 383.5), Vec2(2.0, 386.0), Vec2(4.0, 388.5), Vec2(4.0, 489.0), Vec2(4.0, 589.6), Vec2(5.5, 595.5), Vec2(7.1, 601.5), Vec2(10.5, 606.7), Vec2(14.0, 611.9), Vec2(19.0, 615.9), Vec2(24.0, 619.8), Vec2(29.6, 622.5), Vec2(35.2, 625.1), Vec2(41.8, 626.6), Vec2(48.5, 628.0), Vec2(52.0, 628.0), Vec2(55.5, 628.0), Vec2(62.0, 626.5), Vec2(68.5, 624.9), Vec2(78.6, 620.0), Vec2(88.6, 615.2), Vec2(100.1, 605.0), Vec2(111.5, 594.8), Vec2(119.0, 588.7), Vec2(126.5, 582.7), Vec2(127.6, 581.3), Vec2(128.7, 580.0), Vec2(142.2, 571.6), Vec2(154.6, 563.3), Vec2(164.3, 558.1), Vec2(174.0, 552.8), Vec2(169.5, 544.5), Vec2(164.9, 536.8), Vec2(160.5, 528.2), Vec2(156.1, 519.5), Vec2(155.1, 518.1), Vec2(154.2, 516.7), Vec2(149.3, 519.5), Vec2(144.5, 522.3), Vec2(137.5, 527.1), Vec2(130.5, 531.8), Vec2(128.1, 533.7), Vec2(125.8, 535.5), Vec2(116.9, 542.8), Vec2(108.0, 550.0), Vec2(97.7, 560.3), Vec2(87.4, 570.5), Vec2(77.9, 580.7), Vec2(68.5, 590.8), Vec2(64.2, 592.1), Vec2(59.8, 593.4), Vec2(54.2, 592.2), Vec2(48.6, 591.0), Vec2(44.1, 588.8), Vec2(39.6, 586.5), Vec2(37.6, 583.7), Vec2(35.5, 580.9), Vec2(35.2, 477.3), Vec2(35.0, 373.8), Vec2(17.5, 355.6), Vec2(0, 337.5), Vec2(0.1, 334.5), Vec2(0.1, 331.5), Vec2(19.5, 351.5), Vec2(38.9, 371.5), Vec2(39.0, 475.2), Vec2(39.0, 578.8), Vec2(41.5, 581.8), Vec2(44.0, 584.8), Vec2(50.9, 587.0), Vec2(57.7, 589.3), Vec2(61.6, 588.5), Vec2(65.5, 587.8), Vec2(84.5, 568.2), Vec2(103.5, 548.7), Vec2(113.5, 540.3), Vec2(123.5, 531.9), Vec2(133.5, 525.1), Vec2(143.5, 518.3), Vec2(149.1, 515.1), Vec2(154.7, 512.0), Vec2(165.0, 528.1), Vec2(173.5, 544.1), Vec2(177.3, 549.2), Vec2(181.1, 554.2), Vec2(173.6, 557.8), Vec2(166.8, 561.0), Vec2(158.1, 566.2), Vec2(149.5, 571.3), Vec2(140.3, 577.7), Vec2(131.1, 584.1), Vec2(120.3, 592.9), Vec2(109.5, 601.7), Vec2(100.8, 609.7), Vec2(92.1, 617.7), Vec2(81.8, 622.9), Vec2(71.5, 628.0), Vec2(63.6, 630.0), Vec2(55.7, 632.0), Vec2(51.6, 631.9), Vec2(47.5, 631.8), Vec2(41.0, 630.4)];
        //PL.addShape(table, x, y, vs, 2);
        // right ramp side 1
        vs = [Vec2(414.7, 663.5), Vec2(410.8, 662.0), Vec2(406.0, 658.5), Vec2(401.1, 654.9), Vec2(397.2, 650.2), Vec2(393.3, 645.5), Vec2(389.5, 639.5), Vec2(385.7, 633.5), Vec2(384.9, 633.2), Vec2(384.0, 632.8), Vec2(384.0, 631.8), Vec2(360.5, 594.3), Vec2(337.0, 557.8), Vec2(361.6, 547.6), Vec2(386.1, 538.8), Vec2(403.6, 576.9), Vec2(420.5, 614.3), Vec2(420.8, 492.9), Vec2(421.1, 371.5), Vec2(440.5, 351.5), Vec2(459.9, 331.5), Vec2(459.9, 334.5), Vec2(460.0, 337.5), Vec2(442.5, 355.7), Vec2(425.0, 373.8), Vec2(424.8, 500.6), Vec2(424.5, 627.3), Vec2(423.4, 627.7), Vec2(422.3, 628.1), Vec2(404.0, 587.5), Vec2(385.6, 547.0), Vec2(384.0, 545.4), Vec2(384.0, 543.8), Vec2(382.8, 544.3), Vec2(381.5, 544.7), Vec2(362.3, 551.6), Vec2(343.0, 558.5), Vec2(362.5, 590.0), Vec2(382.1, 620.4), Vec2(382.8, 621.0), Vec2(383.6, 621.5), Vec2(384.3, 623.3), Vec2(385.0, 625.2), Vec2(392.4, 636.3), Vec2(399.7, 647.5), Vec2(404.9, 652.3), Vec2(410.0, 657.1), Vec2(414.3, 659.0), Vec2(418.5, 661.0), Vec2(423.5, 661.0), Vec2(428.5, 661.0), Vec2(433.5, 658.6), Vec2(438.5, 656.3), Vec2(443.6, 651.7), Vec2(448.7, 647.1), Vec2(452.1, 640.8), Vec2(455.5, 634.5), Vec2(455.8, 511.5), Vec2(456.1, 388.5), Vec2(458.0, 386.0), Vec2(460.0, 383.5), Vec2(459.7, 509.0), Vec2(459.5, 634.5), Vec2(456.1, 641.4), Vec2(452.8, 648.2), Vec2(447.3, 653.5), Vec2(441.9, 658.9), Vec2(435.8, 662.0), Vec2(429.7, 665.0), Vec2(424.1, 664.9), Vec2(418.5, 664.9), Vec2(414.7, 663.5)];
        //PL.addShape(table, x, y, vs, 2);

        // left rail straight
        vs = [Vec2(54.9, 623.3), Vec2(50.3, 621.8), Vec2(45.4, 618.1), Vec2(40.5, 614.5), Vec2(36.3, 608.3), Vec2(32.2, 602.1), Vec2(29.9, 596.3), Vec2(27.6, 590.5), Vec2(27.5, 581.5), Vec2(27.5, 572.5), Vec2(29.7, 567.8), Vec2(31.9, 563.1), Vec2(36.6, 557.9), Vec2(41.4, 552.8), Vec2(58.5, 539.0), Vec2(75.5, 525.2), Vec2(84.7, 516.5), Vec2(94.0, 507.9), Vec2(99.9, 500.4), Vec2(105.9, 493.0), Vec2(109.9, 486.2), Vec2(113.9, 479.5), Vec2(115.9, 473.0), Vec2(117.8, 466.5), Vec2(118.5, 461.8), Vec2(119.1, 457.0), Vec2(121.1, 457.0), Vec2(123.1, 457.0), Vec2(122.4, 462.3), Vec2(121.8, 467.5), Vec2(119.3, 474.9), Vec2(116.9, 482.3), Vec2(113.4, 488.3), Vec2(109.9, 494.3), Vec2(105.2, 500.4), Vec2(100.5, 506.5), Vec2(91.6, 515.4), Vec2(82.7, 524.2), Vec2(71.1, 533.9), Vec2(59.4, 543.5), Vec2(49.2, 551.8), Vec2(39.1, 560.1), Vec2(36.0, 564.8), Vec2(33.0, 569.4), Vec2(31.9, 573.4), Vec2(30.8, 577.5), Vec2(31.3, 584.3), Vec2(31.7, 591.1), Vec2(35.9, 599.5), Vec2(40.0, 607.8), Vec2(45.5, 612.8), Vec2(50.9, 617.8), Vec2(55.6, 619.4), Vec2(60.4, 621.0), Vec2(66.1, 621.0), Vec2(71.8, 621.0), Vec2(78.1, 619.0), Vec2(84.5, 617.0), Vec2(88.6, 614.3), Vec2(92.8, 611.5), Vec2(103.6, 601.8), Vec2(114.5, 592.2), Vec2(121.0, 587.0), Vec2(127.5, 581.8), Vec2(127.8, 580.9), Vec2(128.2, 580.0), Vec2(142.2, 571.6), Vec2(154.6, 563.3), Vec2(164.3, 558.1), Vec2(174.0, 552.8), Vec2(169.7, 544.8), Vec2(165.3, 537.5), Vec2(160.1, 527.3), Vec2(154.8, 517.0), Vec2(136.3, 528.8), Vec2(119.1, 540.5), Vec2(108.0, 550.4), Vec2(96.9, 560.3), Vec2(83.3, 575.2), Vec2(69.7, 590.0), Vec2(66.1, 587.6), Vec2(63.8, 585.1), Vec2(64.3, 582.8), Vec2(64.9, 580.4), Vec2(78.1, 569.0), Vec2(91.4, 557.5), Vec2(99.9, 549.6), Vec2(108.5, 541.7), Vec2(118.3, 531.2), Vec2(128.0, 520.6), Vec2(128.0, 519.4), Vec2(128.0, 518.2), Vec2(129.3, 517.7), Vec2(130.6, 517.2), Vec2(135.7, 509.9), Vec2(140.8, 502.5), Vec2(144.8, 494.5), Vec2(148.9, 486.5), Vec2(151.0, 480.0), Vec2(153.2, 473.5), Vec2(154.3, 465.3), Vec2(155.4, 457.0), Vec2(157.4, 457.0), Vec2(159.3, 457.0), Vec2(158.6, 463.2), Vec2(158.0, 469.4), Vec2(156.4, 476.0), Vec2(154.8, 482.5), Vec2(150.0, 492.6), Vec2(145.2, 502.8), Vec2(140.9, 509.1), Vec2(136.7, 515.5), Vec2(132.1, 521.5), Vec2(127.5, 527.5), Vec2(126.3, 528.3), Vec2(125.0, 529.1), Vec2(135.8, 523.6), Vec2(146.6, 516.5), Vec2(150.9, 513.7), Vec2(155.3, 510.9), Vec2(156.1, 511.7), Vec2(156.9, 512.5), Vec2(167.1, 531.0), Vec2(177.2, 549.5), Vec2(179.3, 551.8), Vec2(181.4, 554.0), Vec2(180.4, 554.9), Vec2(179.5, 555.7), Vec2(158.0, 570.7), Vec2(136.5, 585.6), Vec2(134.9, 586.2), Vec2(133.3, 586.8), Vec2(117.4, 598.0), Vec2(101.5, 609.3), Vec2(97.3, 613.1), Vec2(93.2, 616.9), Vec2(86.7, 619.9), Vec2(80.2, 622.9), Vec2(75.9, 623.9), Vec2(71.5, 625.0), Vec2(65.5, 624.9), Vec2(59.5, 624.9), Vec2(54.9, 623.3)];
        PL.addShape(table, x, y, vs, 2);
        // right rail straight
        vs = [Vec2(354.0, 700.6), Vec2(348.5, 699.4), Vec2(343.4, 697.6), Vec2(338.3, 695.9), Vec2(332.4, 692.4), Vec2(326.5, 688.9), Vec2(322.2, 684.7), Vec2(317.8, 680.5), Vec2(313.5, 674.5), Vec2(309.2, 668.5), Vec2(303.7, 657.5), Vec2(298.2, 646.5), Vec2(296.0, 640.0), Vec2(293.8, 633.5), Vec2(293.9, 617.5), Vec2(294.0, 601.5), Vec2(295.0, 573.5), Vec2(296.0, 545.5), Vec2(297.6, 515.0), Vec2(299.2, 484.5), Vec2(299.8, 470.5), Vec2(300.5, 456.5), Vec2(302.8, 456.2), Vec2(305.1, 455.8), Vec2(304.5, 461.7), Vec2(304.0, 467.5), Vec2(302.5, 495.5), Vec2(301.0, 523.5), Vec2(299.9, 549.0), Vec2(298.7, 574.5), Vec2(298.6, 606.0), Vec2(298.5, 637.5), Vec2(302.9, 647.0), Vec2(307.3, 656.5), Vec2(311.9, 664.5), Vec2(316.5, 672.4), Vec2(323.3, 679.6), Vec2(330.0, 686.8), Vec2(336.9, 690.4), Vec2(343.8, 694.0), Vec2(352.6, 696.1), Vec2(361.5, 698.2), Vec2(370.5, 697.0), Vec2(379.5, 695.9), Vec2(388.5, 692.9), Vec2(397.5, 689.9), Vec2(403.6, 686.5), Vec2(409.7, 683.1), Vec2(414.6, 678.6), Vec2(419.5, 674.2), Vec2(422.7, 668.8), Vec2(426.0, 663.5), Vec2(427.7, 656.6), Vec2(429.3, 649.8), Vec2(428.2, 644.0), Vec2(427.0, 638.3), Vec2(406.4, 592.9), Vec2(385.9, 547.5), Vec2(384.9, 547.2), Vec2(384.0, 546.8), Vec2(384.0, 545.3), Vec2(384.0, 543.8), Vec2(382.8, 544.3), Vec2(381.5, 544.7), Vec2(362.3, 551.6), Vec2(343.0, 558.5), Vec2(362.5, 590.1), Vec2(382.1, 620.5), Vec2(383.0, 620.8), Vec2(384.0, 621.2), Vec2(384.0, 622.3), Vec2(384.0, 623.5), Vec2(391.4, 634.9), Vec2(398.8, 646.3), Vec2(396.6, 649.9), Vec2(394.5, 653.5), Vec2(389.8, 658.3), Vec2(385.0, 663.1), Vec2(380.7, 665.0), Vec2(376.5, 667.0), Vec2(370.8, 667.0), Vec2(365.0, 667.0), Vec2(358.6, 664.6), Vec2(352.3, 662.2), Vec2(348.3, 659.7), Vec2(344.3, 657.2), Vec2(340.7, 652.9), Vec2(337.0, 648.7), Vec2(334.9, 643.1), Vec2(332.8, 637.5), Vec2(331.4, 630.0), Vec2(329.9, 622.5), Vec2(329.3, 597.0), Vec2(328.8, 571.5), Vec2(330.9, 537.5), Vec2(333.1, 503.5), Vec2(334.8, 480.5), Vec2(336.5, 457.5), Vec2(338.7, 457.2), Vec2(340.9, 456.9), Vec2(340.4, 460.2), Vec2(340.0, 463.5), Vec2(338.0, 491.5), Vec2(336.0, 519.5), Vec2(334.4, 545.5), Vec2(332.8, 571.5), Vec2(333.3, 595.0), Vec2(333.8, 618.5), Vec2(335.4, 626.8), Vec2(337.1, 635.0), Vec2(339.0, 639.4), Vec2(340.9, 643.8), Vec2(344.7, 648.1), Vec2(348.5, 652.3), Vec2(355.5, 655.5), Vec2(362.5, 658.7), Vec2(367.7, 660.0), Vec2(372.9, 661.2), Vec2(378.2, 659.6), Vec2(383.5, 658.0), Vec2(384.5, 657.0), Vec2(385.5, 656.0), Vec2(388.0, 654.1), Vec2(390.5, 652.2), Vec2(392.2, 649.3), Vec2(393.9, 646.4), Vec2(389.8, 639.9), Vec2(385.8, 633.5), Vec2(384.9, 633.2), Vec2(384.0, 632.8), Vec2(384.0, 631.8), Vec2(360.5, 594.3), Vec2(337.0, 557.8), Vec2(361.6, 547.6), Vec2(386.1, 538.8), Vec2(408.8, 588.5), Vec2(430.9, 637.5), Vec2(432.1, 642.3), Vec2(433.3, 647.1), Vec2(432.8, 653.9), Vec2(432.3, 660.8), Vec2(429.3, 667.2), Vec2(426.3, 673.6), Vec2(420.9, 678.9), Vec2(415.6, 684.2), Vec2(409.0, 688.0), Vec2(402.4, 691.9), Vec2(394.9, 694.8), Vec2(387.5, 697.8), Vec2(380.5, 699.3), Vec2(373.5, 700.9), Vec2(366.5, 701.4), Vec2(359.5, 701.9), Vec2(354.0, 700.6)];
        PL.addShape(table, x, y, vs, 2);
    }
}
