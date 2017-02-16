//  ******************* Tango dancer 3D 2016 ***********************
Boolean 
  animating=true,
  PickedFocus=false, 
  center=true, 
  track=false, 
  showViewer=false, 
  showBalls=false, 
  showControl=true, 
  showCurve=true, 
  showPath=true, 
  showKeys=true, 
  showSkater=true, 
  scene1=false,
  solidBalls=false,
  showCorrectedKeys=true,
  showQuads=true,
  showVecs=true,
  showTube=false;
String instruction = "Scroll up to zoom out!";
float 
  t=0, 
  s=0,
  b=0,
  grav= -100;
int
  f=0, maxf=2*30, level=4, method=3;
String SDA = "angle";
float defectAngle=0;
pts P = new pts(); // polyloop in 3D
pts Q = new pts(); // second polyloop in 3D
pts R = new pts(); // inbetweening polyloop L(P,t,Q);
pts R2 = new pts();

pt hip = new pt(), knee = new pt(), ankle = new pt(), heel = new pt(), ball = new pt(), toe = new pt();
pt kneeL = new pt(), ankleL = new pt(), heelL = new pt(), ballL = new pt(), toeL = new pt();
pt bod = new pt(); pt neck = new pt(); pt head = new pt();
float rHip=17, rKnee=10, rFeetParts=3; // radii of Hip, Knee, Ankle, hEel, Ball, Toe
float rBod = 25; float rHead = 15; float rNeck = 10;
//float _rH=200, _rK=20, _rA=20, _rE=25, _rB=15, _rT=5; // radii of Hip, Knee, Ankle, hEel, Ball, Toe
// leg measures (to update press '/' and copy print here):
float hipKnee=20, kneeAnkle=20, ankleHeel=7, heelBall=15, ankleBall=sqrt(sq(ankleHeel)+sq(heelBall)), ballToe=5;
float upperBodyHeight=40, neckLength = 7;//body height
float hipHeight; // height of _H
float legSpread = -2*grav;
float hipAngle=PI/3;
float kneeAngle=PI/4;
float ankleAngle=5*PI/9;

vec feetVec = new vec();
vec forwardVec = new vec();

  
void setup() {
  //myFace = loadImage("data/pic.jpg");  // load image from file pic.jpg in folder data *** replace that file with your pic of your own face
  //myFace = loadImage("data/pic.jpg");
  textureMode(NORMAL);         
  size(675, 675, P3D); // P3D means that we will do 3D graphics
  P.declare(); Q.declare(); R.declare(); // P is a polyloop in 3D: declared in pts
  //P.resetOnCircle(6,100); Q.copyFrom(P); // use this to get started if no model exists on file: move points, save to file, comment this line
  P.loadPts("data/pts");  Q.loadPts("data/pts2"); // loads saved models from file (comment out if they do not exist yet)
  //set R to Q once
  noSmooth();
  frameRate(30);
  Q.magnify(4);
  R.copyFrom(Q);
  R2.copyFrom(R);
  }

void draw() {
  background(255);
  hint(ENABLE_DEPTH_TEST); 
  pushMatrix();   // to ensure that we can restore the standard view before writing on the canvas
  setView();  // see pick tab
  showFloor(); // draws dance floor as yellow mat
  doPick(); // sets Of and axes for 3D GUI (see pick Tab)
  P.SETppToIDofVertexWithClosestScreenProjectionTo(Mouse()); // for picking (does not set P.pv)
  
   
  //for(int i=0; i<level; i++) 
    //{
    //Q.copyFrom(R);
    //R2.copyFrom(R);
    fill(blue); if(showCurve) Q.drawClosedCurve(3);
    if(animating) {
      //if(method==4) {Q.subdivideDemoInto(R);}
      if(method==3) {R2.subdivideQuinticInto(R); }
      if(method==2) {R2.subdivideCubicInto(R); }
      //if(method==2) {Q.subdivideJarekInto(R); }
      //if(method==2) {R2.subdivideDemoInto(R); }
      if(method==1) {R2.subdivideFourPointInto(R); }
      if(method==0) {R2.subdivideQuadraticInto(R);}
    }
    //}
  R.displaySkater();
  
  
  if(showControl) {fill(grey); Q.drawClosedCurve(3);}  // draw control polygon 
  fill(yellow,100); Q.showPicked(); 
  

  //if(animating)  
  //  {
  //  f++; // advance frame counter
  //  if (f>maxf) // if end of step
  //    {
  //    P.next();     // advance dv in P to next vertex
 ////     animating=true;  
  //    f=0;
  //    }
  //  }
  //t=(1.-cos(PI*f/maxf))/2; //t=(float)f/maxf;

  //if(track) F=_LookAtPt.move(X(t)); // lookAt point tracks point X(t) filtering for smooth camera motion (press'.' to activate)
 
  popMatrix(); // done with 3D drawing. Restore front view for writing text on canvas
  hint(DISABLE_DEPTH_TEST); // no z-buffer test to ensure that help text is visible
    if(method==3) scribeHeader("Quintic UBS",2);
    if(method==2) scribeHeader("Cubic UBS",2);
    //if(method==2) scribeHeader("Jarek J-spline",2);
    if(method==1) scribeHeader("Four Points",2);
    if(method==0) scribeHeader("Quadratic UBS",2);
    
    
    //scribeHeader("press a -> First subdivision and start animation",3);
    scribeHeader("press x -> Another subdivision                     "+instruction,4);
    scribeHeader("press m -> change type of sibdivision",5);
    scribeHeader("press r -> restart program",6);

  // used for demos to show red circle when mouse/key is pressed and what key (disk may be hidden by the 3D model)
  if(keyPressed) {stroke(red); fill(white); ellipse(mouseX,mouseY,26,26); fill(red); text(key,mouseX-5,mouseY+4);}
  if(scribeText) {fill(black); displayHeader();} // dispalys header on canvas, including my face
  if(scribeText && !filming) displayFooter(); // shows menu at bottom, only if not filming
  if(filming && (animating || change)) saveFrame("FRAMES/F"+nf(frameCounter++,4)+".tif");  // save next frame to make a movie
  change=false; // to avoid capturing frames when nothing happens (change is set uppn action)
  }