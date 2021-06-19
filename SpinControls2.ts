/**
 * @author Eberhard Graether / http://egraether.com/
 * @author Mark Lundin 	/ http://mark-lundin.com
 * @author Simone Manini / http://daron1337.github.io
 * @author Luca Antiga 	/ http://lantiga.github.io
 * @author Paul Elliott / http://vizworkshop.com
 */
import {
    Camera,
    Object3D,
    Quaternion,
    Vector3,
    EventDispatcher,
    Vector2,
    PerspectiveCamera,
    Ray,
    Sphere,
    OrthographicCamera,
} from 'three'

export class SpinControls extends EventDispatcher{
    public object: Object3D
    public trackballRadius: number
    public camera: Camera
    public domElement: HTMLElement
    public enabled: boolean
    public rotateSensitivity: number
    public relativelySpinOffTrackball: boolean
    public enableDamping: boolean
    public dampingFactor: number
    public spinAxisConstraint?: Vector3

    // Raycast projects pointer line through camera frustum for accurate trackball control.
    // Shoemake has direct touching feel of pointer on orthographically projected sphere but jumps at sphere edge.
    // Holyroyd smooths between sphere and hyperbola to avoid jump at sphere edge.
    // Azimuthal from Yasuhiro Fujii has unlimited rotation behond the sphere edge.
    POINTER_SPHERE_MAPPING = { SHOEMAKE: 'shoemake', HOLROYD: 'holroyd',
        AZIMUTHAL: 'azimuthal', RAYCAST: 'raycast' };

    // Base this on angle change around sphere edge?
    offTrackBallVelocityGainMap: Record<string, number> = {
        'shoemake': 20,
        'holroyd': 8,
        'azimuthal': 8,
        'raycast': 20
    };

    // Internals
    private _pointerMapping = this.POINTER_SPHERE_MAPPING.RAYCAST;
    private _offTrackBallVelocityGain = this.offTrackBallVelocityGainMap[this._pointerMapping];
    private _pointerUpVelDamping = 2000;

    screen = { left: 0, top: 0, width: 0, height: 0 };

    private _angularVelocity = new Vector3(0, 0, 0);
    private _lastQuaternion = new Quaternion();
    private _lastVelTime = 0;

    private _pointOnSphere = new Vector3();
    private _pointerScreen = new Vector2();
    private _pointOnSphereOld = new Vector3();
    private _lastPointerEventTime = 0;
    private _wasLastPointerEventOnSphere = false;

    private _isPointerDown = false;

    private _EPS = 0.000001;

    changeEvent = { type: 'change' };
    startEvent = { type: 'start' };
    endEvent = { type: 'end' };
    private hasPointerMovedThisFrame: boolean = false

    constructor(object: Object3D, trackBallRadius: number, camera: Camera, domElement: HTMLElement) {
        super()
        this.object = object;
        this.trackballRadius = trackBallRadius;
        this.camera = camera;
        this.domElement = ( domElement !== undefined ) ? domElement : document.body;

        // API

        this.enabled = true;

        this.rotateSensitivity = 1.0; // Keep at 1 for direct touching feel
        this.relativelySpinOffTrackball = true; // Rotation continues relatively when pointer is beyond trackball
        this.enableDamping = true; // True for movement with momentum after pointer release on control.update
        this.dampingFactor = 5; // Increase for more friction
        this.spinAxisConstraint = undefined; // Set to a Vector3 to limit spinning to about an axis

        this.onMouseDown = this.onMouseDown.bind(this);
        this.onTouchStart = this.onTouchStart.bind(this);
        this.onTouchMove = this.onTouchMove.bind(this);
        this.onMouseMove = this.onMouseMove.bind(this);
        this.onMouseUp = this.onMouseUp.bind(this);

        this.domElement.addEventListener( 'pointerdown', this.onMouseDown );

        this.domElement.addEventListener( 'touchstart', this.onTouchStart, {passive: false} );
        this.domElement.addEventListener( 'touchmove', this.onTouchMove, {passive: false} );
        this.domElement.addEventListener( 'touchend', this.onTouchEnd, {passive: false} );

        this.onWindowResize();
        // force an update at start
        this.update();
    }

    private _update_currentTime = 0;
    private _update_lastTime = performance.now() / 1000.0;
    private _update_deltaTime = 0;
    update() {
        this._update_currentTime = performance.now() / 1000.0;
        this._update_deltaTime = this._update_currentTime - this._update_lastTime;
        this._update_lastTime = this._update_currentTime;

        if( !this._isPointerDown && this.enableDamping ) {

            this._angularVelocity.multiplyScalar( 1 / ( this._update_deltaTime * this.dampingFactor + 1 ) );

            this.applyVelocity();

        }

        if( !this.enableDamping ) {

            this._lastVelTime = performance.now(); // ToDo Avoid this hack.  Causes trackball drift.

        }

        this.hasPointerMovedThisFrame = false;

    }

    private _updateAngularVelocity_q0 = new Quaternion();
    private _updateAngularVelocity_q1 = new Quaternion();
    private _updateAngularVelocity_q0Conj = new Quaternion(); //for path independent rotation

    updateAngularVelocity(p1: Vector3, p0: Vector3, timeDelta: number) {

        // return function updateAngularVelocity(  ) {

            // path independent rotation from Shoemake
            this._updateAngularVelocity_q0Conj.set(p0.x, p0.y, p0.z, 0.0)
            this._updateAngularVelocity_q0Conj.normalize();
            this._updateAngularVelocity_q0Conj.conjugate();
            this._updateAngularVelocity_q1.set(p1.x, p1.y, p1.z, 0.0).multiply(this._updateAngularVelocity_q0Conj);
            timeDelta *= 2.0; // divide angleDelta by 2 to keep sphere under pointer.  Might break algorithm properties, TODO: perhaps investigate.

            // path dependent
            // q1.setFromUnitVectors(p0, p1);

        this._updateAngularVelocity_q0.set(p0.x, p0.y, p0.z, 1.0);
            let angleSpeed = this._updateAngularVelocity_q1.angleTo(this._updateAngularVelocity_q0) / timeDelta;

            // Just set velocity because we are touching trackball without sliding
            this._angularVelocity.crossVectors( p0, p1);
            this._angularVelocity.setLength( angleSpeed );
            this.applyVelocity();
        //
        // };

    }

    private _applyVelocity_quat = new Quaternion();
    private _applyVelocity_normalizedAxis = new Vector3();

    applyVelocity() {

            let timeStamp = performance.now();
            let deltaTime = ( timeStamp - this._lastVelTime ) / 1000.0;

            this._lastVelTime = timeStamp;
            let deltaAngle;
            if( this.spinAxisConstraint ) {

                this._applyVelocity_normalizedAxis.copy( this.spinAxisConstraint );
                deltaAngle = this._applyVelocity_normalizedAxis.dot( this._angularVelocity ) ;

            } else {

                this._applyVelocity_normalizedAxis.copy( this._angularVelocity );
                deltaAngle = this._angularVelocity.length();

            }

            if ( deltaAngle && deltaTime ) {

                this._applyVelocity_normalizedAxis.normalize();
                this._applyVelocity_quat.setFromAxisAngle( this._applyVelocity_normalizedAxis, deltaAngle * deltaTime * this.rotateSensitivity );

                this.object.quaternion.normalize();
                this.object.quaternion.premultiply(this._applyVelocity_quat);

                // using small-angle approximation cos(x/2) = 1 - x^2 / 8

                if ( 8 * ( 1 - this._lastQuaternion.dot( this.object.quaternion ) ) > this._EPS) {

                    console.log(deltaTime, deltaAngle, this._applyVelocity_quat)
                    this.dispatchEvent( this.changeEvent );

                    this._lastQuaternion.copy( this.object.quaternion );

                }

            }

    }

    onWindowResize() {

        if ( this.domElement === document.body ) {

            this.screen.left = 0;
            this.screen.top = 0;
            this.screen.width = window.innerWidth;
            this.screen.height = window.innerHeight;

        } else {

            var box = this.domElement.getBoundingClientRect();
            var d = this.domElement.ownerDocument.documentElement;
            this.screen.left = box.left + window.pageXOffset - d.clientLeft;
            this.screen.top = box.top + window.pageYOffset - d.clientTop;
            this.screen.width = box.width;
            this.screen.height = box.height;

        }

    }


    resetInputAfterCameraMovement() {

        if( this._isPointerDown ) {

            // Need to update camera.matrixWorldInverse if camera is moved
            // and renderer has not updated matrixWorldInverse yet.
            this.camera.updateWorldMatrix(true, false);
            this.camera.matrixWorldInverse.copy( this.camera.matrixWorld ).invert();

            this._pointOnSphere.copy( this.getPointerInSphere( this.getPointerInNdc( this._pointerScreen.x, this._pointerScreen.y ) ) );
        }

    }
    getPointerInNdc(pageX: number, pageY: number) {
            return new Vector2(
                ( pageX - this.screen.width * 0.5 - this.screen.left ) / ( this.screen.width * 0.5 ),
                ( this.screen.height + 2 * ( this.screen.top - pageY ) ) / this.screen.height
            );
        };

    private _getObjectToPointer_objPos = new Vector3();
    private _getObjectToPointer_objEdgePos = new Vector3();
    private _getObjectToPointer_offset = new Vector3();
    private _getObjectToPointer_objToPointer = new Vector2();
    private _getObjectToPointer_cameraRot = new Quaternion();

    // Find vector from object to pointer in screen space
    getObjectToPointer(pointerNdcScreen: Vector2) {

            this.object.updateWorldMatrix( true, false );
        this._getObjectToPointer_objPos.setFromMatrixPosition( this.object.matrixWorld );
            this.camera.updateWorldMatrix( true, false );
            // Need to update camera.matrixWorldInverse if camera moved before renderer.render
            this.camera.matrixWorldInverse.copy( this.camera.matrixWorld ).invert();
            this._getObjectToPointer_objPos.project( this.camera ); // position in ndc/screen
        this._getObjectToPointer_objToPointer.set( this._getObjectToPointer_objPos.x, this._getObjectToPointer_objPos.y );
        this._getObjectToPointer_objToPointer.subVectors( pointerNdcScreen, this._getObjectToPointer_objToPointer );

            // Normalize objToPointer by object screen size
            // so objToPointer of length 1 is 1 object radius distance from object center.
            this._getObjectToPointer_objEdgePos.setFromMatrixPosition( this.object.matrixWorld ); // objEdgePos is still aspirational on this line
        this._getObjectToPointer_offset.set( this.trackballRadius, 0, 0 );

        this._getObjectToPointer_offset.applyQuaternion( this._getObjectToPointer_cameraRot.setFromRotationMatrix( this.camera.matrixWorld ) );
            this._getObjectToPointer_objEdgePos.add( this._getObjectToPointer_offset );
            this._getObjectToPointer_objEdgePos.project( this.camera ); // position in ndc/screen
            this._getObjectToPointer_objEdgePos.z = 0;
        this._getObjectToPointer_objPos.z = 0;
            var objRadiusNDC = this._getObjectToPointer_objEdgePos.distanceTo( this._getObjectToPointer_objPos );

        this._getObjectToPointer_objToPointer.x /= objRadiusNDC;
        this._getObjectToPointer_objToPointer.y /= objRadiusNDC;
            if ( (this.camera as PerspectiveCamera).isPerspectiveCamera  && (this.camera as PerspectiveCamera).aspect ) { // Perspective camera probably
                this._getObjectToPointer_objToPointer.y /= (this.camera as PerspectiveCamera).aspect;
            }

            return this._getObjectToPointer_objToPointer;

    }

    private _getPointerInSphere_point = new Vector3();
    private _getPointerInSphere_objPos = new Vector3();
    private _getPointerInSphere_objToPointer = new Vector2();
    private _getPointerInSphere_cameraRot = new Quaternion();
    private _getPointerInSphere_trackBallSphere = new Sphere();
    private _getPointerInSphere_ray = new Ray();

    // Finds point on sphere in world coordinate space
    getPointerInSphere(ndc: Vector2) {

            this._getPointerInSphere_objToPointer.copy( this.getObjectToPointer( ndc ) );

            this._getPointerInSphere_cameraRot.setFromRotationMatrix( this.camera.matrixWorld );

            if ( this._pointerMapping === this.POINTER_SPHERE_MAPPING.RAYCAST ) {

                if ( this._getPointerInSphere_objToPointer.lengthSq() < 1 ) {

                    this._getPointerInSphere_objPos.setFromMatrixPosition( this.object.matrixWorld );
                    this._getPointerInSphere_trackBallSphere.set( this._getPointerInSphere_objPos, this.trackballRadius );

                    this._getPointerInSphere_ray.origin.copy( this.camera.position );
                    this._getPointerInSphere_ray.direction.set( ndc.x, ndc.y, .5 );
                    this._getPointerInSphere_ray.direction.unproject( this.camera ); // In world space
                    this._getPointerInSphere_ray.direction.sub( this.camera.position ).normalize(); // Subtract to put around origin

                    this._getPointerInSphere_ray.intersectSphere( this._getPointerInSphere_trackBallSphere, this._getPointerInSphere_point );
                    this._getPointerInSphere_point.sub( this._getPointerInSphere_objPos );
                    this._getPointerInSphere_point.normalize(); // updateAngularVelocity expects unit vectors

                } else {

                    // Shoemake project on edge of sphere
                    this._getPointerInSphere_objToPointer.normalize();
                    this._getPointerInSphere_point.set( this._getPointerInSphere_objToPointer.x, this._getPointerInSphere_objToPointer.y, 0.0 );
                    this._getPointerInSphere_point.applyQuaternion( this._getPointerInSphere_cameraRot );

                }

            }
            // Pointer mapping code below derived from Yasuhiro Fujii's https://mimosa-pudica.net/3d-rotation/
            else if ( this._pointerMapping === this.POINTER_SPHERE_MAPPING.HOLROYD ) {

                var t = this._getPointerInSphere_objToPointer.lengthSq();
                if (t < 0.5) {
                    this._getPointerInSphere_point.set( this._getPointerInSphere_objToPointer.x, this._getPointerInSphere_objToPointer.y, Math.sqrt( 1.0 - t ) );
                } else {
                    this._getPointerInSphere_point.set( this._getPointerInSphere_objToPointer.x, this._getPointerInSphere_objToPointer.y, 1.0 / ( 2.0 * Math.sqrt( t ) ) );
                    this._getPointerInSphere_point.normalize();
                }
                this._getPointerInSphere_point.applyQuaternion( this._getPointerInSphere_cameraRot ); // Rotate from looking down z axis to camera direction

            } else if ( this._pointerMapping === this.POINTER_SPHERE_MAPPING.SHOEMAKE ) {

                var t = this._getPointerInSphere_objToPointer.lengthSq();
                if (t < 1.0) {
                    this._getPointerInSphere_point.set( this._getPointerInSphere_objToPointer.x, this._getPointerInSphere_objToPointer.y, Math.sqrt( 1.0 - t ) );
                } else {
                    this._getPointerInSphere_objToPointer.normalize();
                    this._getPointerInSphere_point.set( this._getPointerInSphere_objToPointer.x, this._getPointerInSphere_objToPointer.y, 0.0 );
                }
                this._getPointerInSphere_point.applyQuaternion( this._getPointerInSphere_cameraRot );

            } else if ( this._pointerMapping === this.POINTER_SPHERE_MAPPING.AZIMUTHAL ) {

                var t = ( Math.PI / 2.0 ) * this._getPointerInSphere_objToPointer.length();
                var sined = t < Number.EPSILON ? 1.0 : Math.sin( t ) / t;
                this._getPointerInSphere_objToPointer.multiplyScalar( ( Math.PI / 2.0 ) * sined );
                this._getPointerInSphere_point.set( this._getPointerInSphere_objToPointer.x, this._getPointerInSphere_objToPointer.y, Math.cos( t ) );
                this._getPointerInSphere_point.applyQuaternion( this._getPointerInSphere_cameraRot );

            }

            return this._getPointerInSphere_point;

    }

    onPointerDown( pointerScreenX: number, pointerScreenY: number, time: number ) {

        var pointerNdc = this.getPointerInNdc( pointerScreenX, pointerScreenY );

        var objToPointer = this.getObjectToPointer( pointerNdc );

        if ( objToPointer.lengthSq() < 1 ) {

            this._wasLastPointerEventOnSphere = true;
            this._pointOnSphere.copy( this.getPointerInSphere( pointerNdc ) );

        } else {

            this._wasLastPointerEventOnSphere = false;

        }

        this._pointerScreen.set( pointerScreenX, pointerScreenY );
        this._lastPointerEventTime = time;
        this._angularVelocity.set( 0, 0, 0 );
        this._isPointerDown = true;

    }

    private _onPointerMove_pointerNdc = new Vector2();
    private _onPointerMove_objToPointer = new Vector2();

    // for relative movement off sphere
    private _onPointerMove_deltaMouse = new Vector2();
    private _onPointerMove_lastNdc = new Vector2();
    private _onPointerMove_objectPos = new Vector3();
    private _onPointerMove_objectToCamera = new Vector3();
    private _onPointerMove_polarVel = new Vector3();
    private _onPointerMove_lastPointOnSphere = new Vector3();

    // Finds point on sphere in world coordinate space
    onPointerMove(pointerScreenX: number, pointerScreenY: number, time: number) {

            let pointerNdc = this._onPointerMove_pointerNdc;
            let objToPointer = this._onPointerMove_objToPointer;

        // for relative movement off sphere
            let deltaMouse = this._onPointerMove_deltaMouse;
            let lastNdc = this._onPointerMove_lastNdc;
            let objectPos = this._onPointerMove_objectPos;
            let objectToCamera = this._onPointerMove_objectToCamera;
            let polarVel = this._onPointerMove_polarVel;
            let lastPointOnSphere = this._onPointerMove_lastPointOnSphere;


            var deltaTime = ( time - this._lastPointerEventTime ) / 1000.0;
            this._lastPointerEventTime = time;

            this._pointOnSphereOld.copy( this._pointOnSphere );

            pointerNdc.copy( this.getPointerInNdc( pointerScreenX, pointerScreenY ) );

            objToPointer.copy( this.getObjectToPointer( pointerNdc ) );

            if ( objToPointer.lengthSq() < 1 || !this.relativelySpinOffTrackball ) {

                // Pointer is within radius of trackball circle on the screen
                // or relative rotation off trackball disabled

                this._pointOnSphere.copy( this.getPointerInSphere( pointerNdc ) );

                if (this._wasLastPointerEventOnSphere ) {

                    // Still on sphere
                    if( deltaTime > 0 ) { // Sometimes zero due to timer precision?

                        this.updateAngularVelocity( this._pointOnSphere, this._pointOnSphereOld, deltaTime );

                    }

                }
                else {

                    // Moved onto sphere
                    this._angularVelocity.set( 0, 0, 0 );
                    this._lastVelTime = time;

                }

                this._wasLastPointerEventOnSphere = true;

            } else {

                // Pointer off trackball

                if ( this._wasLastPointerEventOnSphere ) {

                    // Just moved off trackball

                    this._angularVelocity.set( 0, 0, 0 );
                    this._lastVelTime = time;

                }
                else {

                    // Pointer still off trackball this frame

                    if( deltaTime > 0 ) { // Sometimes zero due to timer precision?

                        // Relatively spin towards pointer from trackball center by change in distance amount
                        // Simplify by finding pointer's delta polar coordinates with Sphere?

                        lastNdc.copy( this.getPointerInNdc( this._pointerScreen.x, this._pointerScreen.y ) );

                        deltaMouse.subVectors(pointerNdc, lastNdc);

                        // Find change in pointer radius to trackball center
                        objectPos.setFromMatrixPosition( this.object.matrixWorld );

                        if ( (this.camera as PerspectiveCamera).isPerspectiveCamera ) {

                            objectToCamera.copy( this.camera.position ).sub( objectPos );

                        } else if((this.camera as OrthographicCamera).isOrthographicCamera){ // Assuming orthographic

                            this.camera.getWorldDirection( objectToCamera );
                            objectToCamera.negate();

                        }

                        this._pointOnSphere.copy( this.getPointerInSphere( pointerNdc ) );

                        // Radius angular velocity direction
                        this._angularVelocity.crossVectors( objectToCamera, this._pointOnSphere );

                        // Find radius change over time

                        var ndcToBall;

                        if ( (this.camera as PerspectiveCamera).isPerspectiveCamera ) {

                            ndcToBall = ( 2 / (this.camera as PerspectiveCamera).fov ) // NDC per field of view degree
                                / Math.atan( this.trackballRadius / objectToCamera.length()); // Ball field of view angle size

                        } else { //Assume orthographic

                            ndcToBall = this.trackballRadius / ( ( (this.camera as OrthographicCamera).top - (this.camera as OrthographicCamera).bottom ) / (this.camera as OrthographicCamera).zoom * 2 );

                        }

                        objToPointer.normalize();
                        var deltaRadius = deltaMouse.dot( objToPointer ) * ndcToBall / deltaTime;
                        this._angularVelocity.setLength( deltaRadius * this._offTrackBallVelocityGain ); // Just set it because we are touching trackball without sliding

                        // Find polar angle change
                        lastPointOnSphere.copy( this.getPointerInSphere( lastNdc ) );
                        let angle = lastPointOnSphere.angleTo( this._pointOnSphere ) / deltaTime;
                        polarVel.crossVectors( lastPointOnSphere, this._pointOnSphere );
                        polarVel.setLength( angle );

                        this._angularVelocity.add( polarVel );

                        this.applyVelocity();

                    }

                }

                this._wasLastPointerEventOnSphere = false;

            }

            this._pointerScreen.set( pointerScreenX, pointerScreenY );

            this.hasPointerMovedThisFrame = true;

    }

    // call like this: spinControl.setPointerToSphereMapping(spinControl.POINTER_SPHERE_MAPPING.SHOEMAKE)
    setPointerToSphereMapping( mappingTechnique: 'shoemake' | 'holroyd' | 'azimuthal' | 'raycast')  {

        this._pointerMapping = mappingTechnique;
        this._offTrackBallVelocityGain = this.offTrackBallVelocityGainMap[this._pointerMapping];

    }

    // listeners

    handlePointerDown( event: Event ) {

        event.preventDefault(); // Prevent the browser from scrolling.
        event.stopImmediatePropagation(); // Stop other controls working.

        // Manually set the focus since calling preventDefault above
        // prevents the browser from setting it automatically.
        this.domElement.focus ? this.domElement.focus() : window.focus();

        this.dispatchEvent( this.startEvent );

    }

    handlePointerUp( event: Event ) {

        event.preventDefault();

        if( !this.hasPointerMovedThisFrame ) {

            // To support subtle touches do big dampening, not just zeroing velocity
            var deltaTime = ( event.timeStamp - this._lastPointerEventTime ) / 1000.0;
            this._angularVelocity.multiplyScalar( 1 / ( this._pointerUpVelDamping * Math.pow(deltaTime, 2) + this.dampingFactor * deltaTime + 1) );

        }

        this._isPointerDown = false;

        this.dispatchEvent( this.endEvent );

    }

    onMouseDown( event: PointerEvent ) {
        if ( this.enabled === false || event.button !== 0 ) return;

        this.onPointerDown( event.pageX, event.pageY, event.timeStamp );

        document.addEventListener( 'pointermove', this.onMouseMove, false );
        document.addEventListener( 'pointerup', this.onMouseUp, false );

        this.handlePointerDown( event );

    }

    onMouseMove( event: PointerEvent ) {

        if ( this.enabled === false ) return;

        event.preventDefault();

        this.onPointerMove( event.pageX, event.pageY, event.timeStamp );

    }

    onMouseUp( event: PointerEvent ) {

        if ( this.enabled === false ) return;

        document.removeEventListener( 'pointermove', this.onMouseMove );
        document.removeEventListener( 'pointerup', this.onMouseUp);

        this.handlePointerUp( event );

    }

    // For camera controls to stop spin with 2 finger pinch
    cancelSpin() {

        this._angularVelocity.set( 0, 0, 0 );

    }

    // Function broken out for CameraSpinControls to use in touch end if going from 2 fingers to 1
    handleTouchStart( event: TouchEvent ) {

        this.onPointerDown( (event as any).pageX, (event as any).pageY, event.timeStamp );
        // this.applyVelocity();  //TODO Should not be needed here

    }

    onTouchStart( event: TouchEvent ) {

        if ( this.enabled === false ) return;

        this.handleTouchStart( event );

        this.handlePointerDown( event );

    }

    onTouchMove( event: TouchEvent ) {

        if ( this.enabled === false || !this._isPointerDown ) return;

        event.preventDefault();
        event.stopImmediatePropagation(); // Prevent other controls from working.

        this.onPointerMove( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY, event.timeStamp );

    }

    onTouchEnd( event: TouchEvent ) {

        if( this.enabled === false ) return;

        this.handlePointerUp( event );

        // override handlePointerUp if finger still down
        if( event.touches.length > 0 ) {
            this._isPointerDown = true;
        }

    }

    dispose () {

        this.domElement.removeEventListener( 'pointerdown', this.onMouseDown );
        document.removeEventListener( 'pointermove', this.onMouseMove );
        document.removeEventListener( 'pointerup', this.onMouseUp );

        this.domElement.removeEventListener( 'touchstart', this.onTouchStart );
        this.domElement.removeEventListener( 'touchmove', this.onTouchMove );
        this.domElement.removeEventListener( 'touchend', this.onTouchEnd );

    };

}
