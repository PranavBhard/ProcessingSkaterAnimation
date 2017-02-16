
class pts // class for manipulaitng and displaying pointclouds or polyloops in 3D 
  { 
    int maxnv = 16000;                 //  max number of vertices
    pt[] G = new pt [maxnv];           // geometry table (vertices)
    char[] L = new char [maxnv];             // labels of points
    vec [] LL = new vec[ maxnv];  // displacement vectors
    Boolean loop=true;          // used to indicate closed loop 3D control polygons
    int pv =0,     // picked vertex index,
        iv=0,      //  insertion vertex index
        dv = 0,   // dancer support foot index
        nv = 0,    // number of vertices currently used in P
        pp=1; // index of picked vertex

  pts() {}
  pts declare() 
    {
    for (int i=0; i<maxnv; i++) G[i]=P(); 
    for (int i=0; i<maxnv; i++) LL[i]=V(); 
    return this;
    }     // init all point objects
  pts empty() {nv=0; pv=0; return this;}                                 // resets P so that we can start adding points
  pts addPt(pt P, char c) { G[nv].setTo(P); pv=nv; L[nv]=c; nv++;  return this;}          // appends a new point at the end
  pts addPt(pt P) { G[nv].setTo(P); pv=nv; L[nv]='f'; nv++;  return this;}          // appends a new point at the end
  pts addPt(float x,float y) { G[nv].x=x; G[nv].y=y; pv=nv; nv++; return this;} // same byt from coordinates
  pts copyFrom(pts Q) {empty(); nv=Q.nv; for (int v=0; v<nv; v++) G[v]=P(Q.G[v]); return this;} // set THIS as a clone of Q
  
  //returns nv
  int size() {
    return nv;
  }
  
  //return point at index i
  pt get(int i) {
    return G[i%nv];
  }
  
  //magnify all coordinates by same float
  void magnify(float f) {
    for (int i=0; i<nv; i++) {
      G[i] = P(f*G[i].x, f*G[i].y, f*G[i].z);
    }
  }
  
    
  
  pts resetOnCircle(int k, float r)  // sets THIS to a polyloop with k points on a circle of radius r around origin
    {
    empty(); // resert P
    pt C = P(); // center of circle
    for (int i=0; i<k; i++) addPt(R(P(C,V(0,-r,0)),2.*PI*i/k,C)); // points on z=0 plane
    pv=0; // picked vertex ID is set to 0
    return this;
    } 
  // ********* PICK AND PROJECTIONS *******  
  int SETppToIDofVertexWithClosestScreenProjectionTo(pt M)  // sets pp to the index of the vertex that projects closest to the mouse 
    {
    pp=0; 
    for (int i=1; i<nv; i++) if (d(M,ToScreen(G[i]))<=d(M,ToScreen(G[pp]))) pp=i; 
    return pp;
    }
  pts showPicked() {show(G[pv],23); return this;}
  pt closestProjectionOf(pt M)    // Returns 3D point that is the closest to the projection but also CHANGES iv !!!!
    {
    pt C = P(G[0]); float d=d(M,C);       
    for (int i=1; i<nv; i++) if (d(M,G[i])<=d) {iv=i; C=P(G[i]); d=d(M,C); }  
    for (int i=nv-1, j=0; j<nv; i=j++) { 
       pt A = G[i], B = G[j];
       if(projectsBetween(M,A,B) && disToLine(M,A,B)<d) {d=disToLine(M,A,B); iv=i; C=projectionOnLine(M,A,B);}
       } 
    return C;    
    }

  // ********* MOVE, INSERT, DELETE *******  
  pts insertPt(pt P) { // inserts new vertex after vertex with ID iv
    for(int v=nv-1; v>iv; v--) {G[v+1].setTo(G[v]);  L[v+1]=L[v];}
     iv++; 
     G[iv].setTo(P);
     L[iv]='f';
     nv++; // increments vertex count
     return this;
     }
  pts insertClosestProjection(pt M) {  
    pt P = closestProjectionOf(M); // also sets iv
    insertPt(P);
    return this;
    }
  pts deletePicked() 
    {
    for(int i=pv; i<nv; i++) 
      {
      G[i].setTo(G[i+1]); 
      L[i]=L[i+1]; 
      }
    pv=max(0,pv-1); 
    nv--;  
    return this;
    }
  pts setPt(pt P, int i) { G[i].setTo(P); return this;}
  
  pts drawBalls(float r) {for (int v=0; v<nv; v++) show(G[v],r); return this;}
  pts showPicked(float r) {show(G[pv],r); return this;}
  pts drawClosedCurve(float r) 
    {
    fill(dgreen);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    fill(magenta);
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    pushMatrix(); //translate(0,0,1); 
    scale(1,1,0.03);  
    fill(grey);
    for (int v=0; v<nv; v++) show(G[v],r*3);    
    for (int v=0; v<nv-1; v++) stub(G[v],V(G[v],G[v+1]),r,r);  
    stub(G[nv-1],V(G[nv-1],G[0]),r,r);
    popMatrix();
    return this;
    }
  pts set_pv_to_pp() {pv=pp; return this;}
  pts movePicked(vec V) { G[pv].add(V); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts setPickedTo(pt Q) { G[pv].setTo(Q); return this;}      // moves selected point (index p) by amount mouse moved recently
  pts moveAll(vec V) {for (int i=0; i<nv; i++) G[i].add(V); return this;};   
  pt Picked() {return G[pv];} 
  pt Pt(int i) {if(0<=i && i<nv) return G[i]; else return G[0];} 

  // ********* I/O FILE *******  
 void savePts(String fn) 
    {
    String [] inppts = new String [nv+1];
    int s=0;
    inppts[s++]=str(nv);
    for (int i=0; i<nv; i++) {inppts[s++]=str(G[i].x)+","+str(G[i].y)+","+str(G[i].z)+","+L[i];}
    saveStrings(fn,inppts);
    };
  
  void loadPts(String fn) 
    {
    println("loading: "+fn); 
    String [] ss = loadStrings(fn);
    String subpts;
    int s=0;   int comma, comma1, comma2;   float x, y;   int a, b, c;
    nv = int(ss[s++]); print("nv="+nv);
    for(int k=0; k<nv; k++) 
      {
      int i=k+s; 
      //float [] xy = float(split(ss[i],",")); 
      String [] SS = split(ss[i],","); 
      G[k].setTo(float(SS[0]),float(SS[1]),float(SS[2]));
      L[k]=SS[3].charAt(0);
      }
    pv=0;
    };
 
  // Dancer
  void setPicekdLabel(char c) {L[pp]=c;}
  


  void setFifo() 
    {
    _LookAtPt.reset(G[dv],60);
    }              


  void next() {dv=n(dv);}
  int n(int v) {return (v+1)%nv;}
  int p(int v) {if(v==0) return nv-1; else return v-1;}
  
  pts subdivideDemoInto(pts Q) 
    {
    Q.empty();
    for(int i=0; i<nv; i++)
      {
      Q.addPt(P(G[i])); 
      Q.addPt(P(G[i],G[n(i)])); 
      //...
      }
    return this;
    }  
  
  void displaySkater() { 
    
    
    
    if(showCurve) {fill(yellow); for (int j=0; j<nv; j++) caplet(G[j],3,G[n(j)],3); }
    //vec[] V = new vec [nv];           // geometry table (vertices)
    pt[] B = new pt [nv];
    vec M = new vec();
    vec totalVec = new vec();
    for (int j=0; j<nv; j++) {
      //V[j]= M(V(0,0,-100), M(V(G[j],G[n(j)]), V(G[p(j)],G[j])));
      
      //creates vec M (acceleration at current -> V(c,n) - V(p,c))
      vec currToNext = V(G[j],G[n(j)]);
      vec prevToCurr = V(G[p(j)],G[j]);
      M = M(currToNext,prevToCurr);
      
      //if(method==4) {Q.subdivideDemoInto(R);}
      if(method==3) {M = V(1.5,M); } //Quintic
      if(method==2) {M = V(1,M); } //Cubic
      //if(method==2) {Q.subdivideJarekInto(R); }
      //if(method==2) {R2.subdivideDemoInto(R); }
      if(method==1) {M = V(0,M); } //Four Point
      //if(method==0) {R2.subdivideQuadraticInto(R);}
      
      totalVec = M(V(0,0,grav), M);
      
      B[j]= P(G[j],totalVec);
     }
      
      
      if(showPath) {fill(cyan); for (int j=0; j<nv; j++) caplet(B[j],3,B[n(j)],3);} 
      if(showKeys) {fill(green); for (int j=0; j<nv; j+=5) arrow(B[j],G[j],3);}
      
      if(animating) f=n(f);
      if(showSkater) {
        //make dancer
        hip = G[f];
        
        
        
        //set feet(balls) at points according to approximate same leg spread -- ATTEMPTING TO SET BOTH FEET ON CURVE CERTAIN DISTANCE APART
        int backDisplace = p(f);
        int frontDisplace = n(f);
        //set back foot approx half length of feet spread from hip projection on B
        while (legSpread > d(B[frontDisplace],B[backDisplace])) {
          backDisplace = p(backDisplace);
          frontDisplace = n(frontDisplace);
        }
        ball = B[backDisplace];
        ballL = B[frontDisplace];
        
        feetVec = V(ball,ballL);
        forwardVec = U(R(feetVec));
        computeSkaterPartsAndDisplay();
        
      }
      else {fill(red); arrow(B[f],G[f],12);} //
      }
  
  pts subdivideQuinticInto(pts R) { //s is equal to 1.5
    R.empty();
    for(int i=0; i<nv; i++) {
      
      pt prev = P(G[p(i)]);
      pt curr = P(G[i]);
      pt next = P(G[n(i)]);
      pt next2 = P(G[n(n(i))]);
      
      pt first = P(1.5,prev,(8-2*(1.5)),curr,1.5,next);
      first.div(8);
      pt second = P(.5,prev,(9-1.5),curr,(9-1.5),next,.5,next2);
      second.div(16);
      
      R.addPt(first);
      R.addPt(second);
      
    }
    return R;
  }
  
  
  pts subdivideCubicInto(pts R) {
    R.empty();
    
    for(int i=0; i<nv; i++) {
      
      pt prev = P(G[p(i)]);
      pt curr = P(G[i]);
      pt next = P(G[n(i)]);
      pt next2 = P(G[n(n(i))]);
      
      pt first = P(1,prev,6,curr,1,next);
      first.div(8);
      pt second = P(0,prev,8,curr,8,next,0,next2);
      second.div(16);
      
      R.addPt(first);
      //println(first);
      R.addPt(second);
      //println(second);
      
    }
    
    return R;
  }
  
  pts subdivideFourPointInto(pts R) {
    R.empty();
    for(int i=0; i<nv; i++) {
      
      pt prev = P(G[p(i)]);
      pt curr = P(G[i]);
      pt next = P(G[n(i)]);
      pt next2 = P(G[n(n(i))]);
      
      pt first = P(0,prev,8,curr,0,next);
      first.div(8);
      pt second = P(-1,prev,9,curr,9,next,-1,next2);
      second.div(16);
      
      R.addPt(first);
      //println(first);
      R.addPt(second);
      //println(second);
      
    }
    return R;
  }
  
  
  pts subdivideQuadraticInto(pts R) {
     
     
     pt curr = new pt();
     pt next = new pt();
     
     R.empty();
     for(int i=0; i<nv; i++) {
       
       curr.setTo(G[i]);
       next.setTo(G[n(i)]);
       
       
       R.addPt(L(curr, .25, next));
       R.addPt(L(curr, .75, next));
     }
     
     
     /*
     int subs = 0;
     while (subs<divisions) {
       subdivideQuadraticInto(R, divisions-subs);
       subs++;
     }
     */
     return R;
     
  }
 
  void capletSection(pt A, float rA, pt B, float rB) { // cone section surface that is tangent to Sphere(A,a) and to Sphere(B,b)
    vec BA = V(B,A);                      //find d btwn centers
    float d = d(B,A);
    float x = ((rB-rA)*rB) / d;           //get x and y values for B1 from B
    float z = sqrt(sq(rB) - sq(x));
    vec xDir = U(BA);
    vec zDir = U(R(BA));
    pt B_p = P(B, x, xDir);
    pt B1 = P(B_p, z, zDir);
    float rB_p = d(B_p, B1);
    pt B2 = P(B_p, -z, zDir);
    
    
    
    //pt Z = MoveByDistanceTowards(B, rB-rA, B1);  //find point Z to make vector
    pt Z = P(B, rB-rA, U(B,B1));
    vec ZA = V(Z,A);                             //point A1 tranformed ZA from point B1
    pt A1 = P(B1, ZA);
    float A1A2Len = rA * (d(B1,B2)/rB);       //Find ratio of radius to A1A2
    vec norm = U(V(B1,B2));
    pt A2 = P(A1, A1A2Len, norm);                //point A2 distance from ratio
    
    float rA_p = d(A1,A2)/2;
    pt A_p = P(A1, rA_p, U(A1,A2));
    
    vec uVecZ = V(0,0,1);
    vec uVecY = V(0,1,0);
    for (float t=0; t<PI; t += PI/30) {
        //make two points per circle per iteration to pass into coneSection
        //Four in total per iteration
        
        pt Point = P(A_p.x, A_p.y, A_p.z + rA_p);
        pt Point2 = P(A_p.x, A_p.y, A_p.z - rA_p);
        pt Pt = P();
        pt Pt2 = P();
        Pt = R(Point, t, uVecZ, uVecY, A_p);
        Pt2 = R(Point2, t, uVecZ, uVecY, A_p);
        
        //Other circle two points
        pt PointB = P(B_p.x, B_p.y, B_p.z + rB_p);
        pt PointB2 = P(B_p.x, B_p.y, B_p.z - rB_p);
        pt PtB = P();
        pt PtB2 = P();
        PtB = R(PointB, t, uVecZ, uVecY, B_p);
        PtB2 = R(PointB2, t, uVecZ, uVecY, B_p);
        /*
        fill(blue, 5);
        line(Pt.x, Pt.y, Pt.z, PtB.x, PtB.y, PtB.z);
        line(Pt2.x, Pt2.y, Pt2.z, PtB2.x, PtB2.y, PtB2.z);
        */
        coneSection(A_p, B_p, rA_p, rB_p);
      }
  }
  
  void computeSkaterPartsAndDisplay() {
    //compute knee
    pt mid = L(ball, .5, hip);
    knee = P(mid, d(hip,knee)/2 * sin(PI/6), forwardVec);
    
    pt midL = L(ballL, .5, hip);
    kneeL = P(midL, d(hip,knee)/2 * sin(PI/6), forwardVec);
    
    /*
    //compute heel, toe -> make spheres for parts
    toe = P(ball, ballToe, forwardVec);
    heel = P(ball, -1 * heelBall, forwardVec);
    
    vec ankleDir = U(R(V(ball, heel)));
    ankle = P(ball, ankleBall, ankleDir);  
    
    toeL = P(ballL, ballToe, forwardVec);
    heelL = P(ballL, -1 * heelBall, forwardVec);
    
    vec ankleDirL = U(R(V(ballL, heelL)));
    ankleL = P(ballL, ankleBall, ankleDirL);
    */
    
    fill(blue); sphere(hip, rHip); 
    fill(blue); sphere(ball, rFeetParts); sphere(ballL, rFeetParts); 
    sphere(knee, rKnee); sphere(kneeL, rKnee);
    
    capletSection(ball, rFeetParts, knee, rKnee);
    capletSection(ballL, rFeetParts, kneeL, rKnee);
    
    capletSection(knee, rKnee, hip, rHip);
    capletSection(kneeL, rKnee, hip, rHip);
    
    /*
    fill(blue); sphere(toe, rFeetParts/2); sphere(ball, rFeetParts);
    sphere(heel, rFeetParts); sphere(ankle, rFeetParts); sphere(knee, rKnee);
    fill(red); sphere(hip, rHip);
    */
    /*
    fill(blue);
    //attach feet, then ankle to knee to hip
    capletSection(toe, rFeetParts/2, ball, rFeetParts);
    capletSection(ball, rFeetParts, heel, rFeetParts);
    capletSection(heel, rFeetParts, ankle, rFeetParts);
    capletSection(ball, rFeetParts, ankle, rFeetParts);
    
    capletSection(ankle, rFeetParts, knee, rKnee);
    capletSection(knee, rKnee, hip, rHip);
    */
    
    fill(green);
    pt UpperBodyCenter = P(hip.x, hip.y, hip.z + rHip + upperBodyHeight);
    capletSection(hip, rHip, UpperBodyCenter, rBod);
    sphere(UpperBodyCenter, rBod);
    
    /*
    
    fill(red);
    capletSection(bod, rBod, HeadStart, rNeck);
    */
    pt HeadStart = P(UpperBodyCenter.x, UpperBodyCenter.y, UpperBodyCenter.z + rBod);
    
    head = P(HeadStart.x,HeadStart.y, HeadStart.z + rHead);
    //fill(red);
    fill(grey);
    capletSection(HeadStart, rNeck, head, rHead);
    sphere(head, rHead);
    
    fill(black);
    pt back = P(head, -3, forwardVec);
    pt hair = P(back.x,back.y,back.z + 5);
    sphere(hair, rHead);
    
  }
        

} // end of pts class