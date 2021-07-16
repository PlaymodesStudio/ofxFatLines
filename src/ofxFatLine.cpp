
#include "ofxFatLine.h"

//--------------------------------------------------------------
ofxFatLine::ofxFatLine(){
    bTriangulation = false;
	joint = OFX_FATLINE_JOINT_BEVEL;
	cap = OFX_FATLINE_CAP_BUTT;
	bFeather = true;
    feathering = 2;
	bFeatherAtCap =true;
	bFeatherAtCore = true;
	bUseGlobalColor = false;
	bUseGlobalWidth = 0;
    //cout << "ofxFatLine" << endl;
}
//--------------------------------------------------------------
void ofxFatLine::setFromPolyline(ofPolyline & poly){
//	ofxFatLine();
	setGlobalColor(ofGetStyle().color);
	setGlobalWidth(ofGetStyle().lineWidth);
	if (!poly.getVertices().empty()){
		addVertices(poly.getVertices());
	for (int i = 0; i <getVertices().size(); i++) {
		addColor(globalColor);
		addWeight(globalWidth);
	}
	update();
	//*/
	}		
}
//--------------------------------------------------------------
ofxFatLine::ofxFatLine(const vector<glm::vec3> &P,const vector<ofFloatColor> &C, const vector<double> &W, bool triangulation){
    ofxFatLine();
    add(P, C, W);
    enableTriangulation(triangulation); 
   // update();
}
//--------------------------------------------------------------
void ofxFatLine::add(const glm::vec3 &thePoint, const ofFloatColor &theColor, const double &theWeight){
    addVertex(thePoint);
    addColor(theColor);
    addWeight(theWeight);
    update();
}
//--------------------------------------------------------------
void ofxFatLine::add(const vector<glm::vec3> &thePoints, const vector<ofFloatColor> &theColors, const vector<double> &theWeights){
    addVertices(thePoints);
    addColors(theColors);
    addWeights(theWeights);
    update();
}
//--------------------------------------------------------------
void ofxFatLine::printDebug(){
    cout << "ofxFatLine: " << endl;
    cout << "Num vertices: " << getVertices().size() << endl;
    cout << "Num colors: " << colors.size() << endl;
    cout << "Num weights: " << weights.size() << endl;
    cout << "Num mesh vertices: " << mesh.getVertices().size() << endl;
    cout << "Num mesh colors: " << mesh.getColors().size() << endl;
    cout << "------------------------------------------------" << endl;
}
//--------------------------------------------------------------
void ofxFatLine::setGlobalColor(ofColor col){
	setGlobalColor(ofFloatColor(col.r, col.g, col.b, col.a));
}
//--------------------------------------------------------------
void ofxFatLine::setGlobalColor(ofFloatColor col){
	globalColor = col;
	bUseGlobalColor = true;
}
//--------------------------------------------------------------
void ofxFatLine::setGlobalWidth(float w){
	bUseGlobalWidth = true;
	globalWidth = w;
}
//--------------------------------------------------------------
void ofxFatLine::updatePoint(int index, glm::vec3 p){
    if (index < getVertices().size()) {
        getVertices()[index] = p;
    }
}
//--------------------------------------------------------------
void ofxFatLine::updateWeight(int index, float w){
    if (index < getVertices().size()) {
        weights.at(index) = w;
    }
}
//--------------------------------------------------------------
float ofxFatLine::getWeight(int index){
    if (index < getVertices().size()) {
        return weights.at(index);
    } else {
        return -1.f;
    }
}
void ofxFatLine::updateColor(int index, ofFloatColor& c){
    if (index < getVertices().size()) {
        colors.at(index) = c;
    }
};
ofColor ofxFatLine::getColor(int index){
    if (index < getVertices().size()) {
        return colors.at(index);
    } else {
        return ofColor();
    }
}
vector<ofFloatColor> ofxFatLine::getColors(){
    return colors;
}
//--------------------------------------------------------------
void ofxFatLine::updateMesh(){
    meshVertices.clear();
    meshColors.clear();
    meshIndices.clear();
//    meshVertices.reserve(getVertices().size() * 10);
//    meshColors.reserve(getVertices().size() * 10);
//    meshIndices.reserve(getVertices().size() * 10);
//    for (int i =0; i<getVertices().size(); i++) {
//        updateVertex(i);
//        /*
//         ofDefaultVec3 a (getVertices()[i-1] - getVertices()[i]);
//         ofDefaultVec3 b (getVertices()[i+1] - getVertices()[i]);
//
//         float  angle = a.angle(b);
//
//         ofDefaultVec3 p = getMidVector(a, b);
//         bool flip = !sameSideOfLine(p, flippepMidVectors.back(), getVertices()[i-1], getVertices()[i]);
//
//         float cs = cos(DEG_TO_RAD * (90 - angle*0.5f));
//         pushNewVertex(getVertices()[i], p, i, cs, flip);
//         //*/
//
//    }
    
    
    int A = 0, B = 0;
    bool on = false;
    if ( getVertices().size() == 0 ) return;
    for (int i = 1; i < getVertices().size() - 1; i++) {
        glm::vec3 V1 (getVertices()[i] - getVertices()[i-1]);
        glm::vec3 V2 (getVertices()[i+1] - getVertices()[i]);
        float len = 0;
//        if (inopt.segmentLength != null) {
//            V1 *= 1 / inopt.segmentLength[i];
//            V2 *= 1 / inopt.segmentLength[i + 1];
//            len += (inopt.segmentLength[i] + inopt.segmentLength[i + 1]) * 0.5f;
//        } else {
            len += glm::length(V1) * 0.5f;
            len += glm::length(V2) * 0.5f;
        
        V1 = glm::normalize(V1);
        V2 = glm::normalize(V2);
//        }
        float costho = V1.x * V2.x + V1.y * V2.y;
        const float m_pi = (float) M_PI;
        //float angle = acos(costho)*180/m_pi;
        float cos_a = (float) cos(15.0f * m_pi / 180.0f);
        float cos_b = (float) cos(10.0f * m_pi / 180.0f);
        float cos_c = (float) cos(25.0f * m_pi / 180.0f);
        float weight = bUseGlobalWidth ? globalWidth : getWeight(i);
        bool approx = false;
        if ((weight * worldToScreenRatio < 7 && costho > cos_a) || (costho > cos_b) || //when the angle difference at an anchor is smaller than a critical degree, do polyline approximation
            (len < weight && costho > cos_c)) { //when vector length is smaller than weight, do approximation
            approx = true;
        }
        if (approx && !on) {
            A = i;
            on = true;
            if (A == 1) {
                A = 0;
            }
            if (A > 1) {
                polylineRange(B, A, false);
            }
        } else if (!approx && on) {
            B = i;
            on = false;
            polylineRange(A, B, true);
        }
    }
    if (on && B < getVertices().size() - 1) {
        B = getVertices().size() - 1;
        polylineRange(A, B, true);
    } else if (!on && A < getVertices().size() - 1) {
        A = getVertices().size() - 1;
        polylineRange(B, A, false);
    }
//    holder = inopt.holder;
//    inopt.holder = null;
    
    
    
    mesh.clear();
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    mesh.addVertices(meshVertices);
    mesh.addColors(meshColors);
    mesh.addIndices(meshIndices);
    // updateMeshIndices();
}
//--------------------------------------------------------------
void ofxFatLine::updateVertex(int index){
    if (index == 0 && getVertices().size() >1){
        updateCap(getVertices()[index+1], getVertices()[index], index);
        
    }else if(index == getVertices().size()-1 && getVertices().size() >1) {
        updateCap(getVertices()[index-1], getVertices()[index], index);
        
    }else if ( getVertices().size() >2){
        glm::vec3 a (getVertices()[index] - getVertices()[index-1]);
        glm::vec3 b (getVertices()[index+1] - getVertices()[index]);
        
        float  angle = glm::angle(a, b);// a.angle(b);
        
        glm::vec3 p = getMidVector(a, b);
        ///Limits midvector length to avoid strange behaviour when angles are small.
        float hyp = MIN(glm::length(a), glm::length(b));
        hyp *= hyp;
        hyp += weights[index] * weights[index];
        hyp = sqrt(hyp);
        //------
        bool flip = !sameSideOfLine(p, flippepMidVectors.back(), getVertices()[index-1], getVertices()[index]);
        
        float cs = cos(angle);
        pushNewVertex(getVertices()[index], p, glm::cross(a, glm::normalize(glm::vec3(0, 0, 1))), glm::cross(b, glm::normalize(glm::vec3(0, 0, 1))), hyp, index, cs, flip);
    }
}
//--------------------------------------------------------------
void ofxFatLine::pushNewAnchor(glm::vec3 a, ofFloatColor c){
    meshVertices.push_back(a);
    meshColors.push_back(c);
}
//--------------------------------------------------------------
void ofxFatLine::pushNewAnchors(glm::vec3 v, glm::vec3 dir, ofFloatColor color, float l1, float l2, bool bInv){
    
    glm::vec3 pp = dir * l1;
    glm::vec3 pa = pp + dir *l2;
    ofFloatColor c(color.r, color.g, color.g, 0);
    if (!bInv) {
        pushNewAnchor(pp + v, color);        
    }
    pushNewAnchor(pa + v, c);
    if (bInv) {
        pushNewAnchor(pp + v, color);
    }
    //    meshVertices.push_back(pp + v);
    //  meshColors.push_back(color);
    
}
//--------------------------------------------------------------
void ofxFatLine::pushNewVertex(glm::vec3 v, glm::vec3 p, glm::vec3 r1, glm::vec3 r2, float maxLength, int index, float cos, bool bFlipped){
    
    ofFloatColor c(colors[index]);
    c.a =0;
    if (cos == 0){
        cos = FLT_EPSILON;
    }
    r1 = glm::normalize(r1);
    r2 = glm::normalize(r2);
    bool bAligned = false;
//    if (abs(cos) == 1) {
//        bAligned = true;
//        p = r1;
//    }
    cos = 1/cos;
    
    midVectors.push_back(p);
    if (bFlipped) {
        p *=-1;
    }
    flippepMidVectors.push_back(p);
    if (bAligned) {
//		cout << "vertexAligned" << endl;
		pushNewAnchors(v, p*-1, colors[index], weights[index], feathering, true);
        pushNewAnchor(v, colors[index]);
        pushNewAnchors(v, p, colors[index], weights[index], feathering, false);
        if (index != 0) {
            if (meshVertices.size() >5) {
                for (int i = meshVertices.size()-4; i<meshVertices.size(); i++) {
                    pushQuadIndices(i);
                }
            }
        }
        
    }else{
        
        if (glm::dot(midVectors.back(), r1) > 0) {
            r1 *=-1;
        }
        if (glm::dot(midVectors.back(), r2) > 0) {
            r2 *=-1;
        }    
        float midLength = weights[index]*cos;
        if (midLength > maxLength) {
            midLength = maxLength;
        }
        if (bFlipped) {
            pushNewAnchors(v, r1, colors[index], weights[index], feathering, !bFlipped);
            pushNewAnchors(v, midVectors.back(), colors[index], midLength, feathering*cos, bFlipped);
            pushNewAnchor(v, colors[index]);
            pushNewAnchors(v, r2, colors[index], weights[index], feathering, !bFlipped);
        }else{
            pushNewAnchors(v, r1, colors[index], weights[index], feathering, !bFlipped);
            pushNewAnchors(v, r2, colors[index], weights[index], feathering, !bFlipped);
            pushNewAnchor(v, colors[index]);
            pushNewAnchors(v, midVectors.back(), colors[index], midLength, feathering*cos, bFlipped);        
        }
        int l = meshVertices.size();
        if (l >11) {
            if (bFlipped) {
                pushQuadIndices(l - 12, l -11, l - 5, l - 4);
                pushQuadIndices(l - 11, l -10, l - 4, l - 3);
                pushQuadIndices(l - 10, l - 9, l - 3, l - 7);
                pushQuadIndices(l -  9, l - 8, l - 7, l - 6);
            }else{
                pushQuadIndices(l - 12, l -11, l - 7, l - 6);
                pushQuadIndices(l - 11, l -10, l - 6, l - 3);
                pushQuadIndices(l - 10, l - 9, l - 3, l - 2);
                pushQuadIndices(l -  9, l - 8, l - 2, l - 1);
            }
        }
        updateJoint(index,bFlipped);
    }

    /*
     ofDefaultVec3 pp = p * 50 * cos;
     ofDefaultVec3 pa = pp + p * 2 * cos;
     
     meshVertices.push_back(pa + v);
     meshColors.push_back(c);
     
     meshVertices.push_back(pp + v);
     meshColors.push_back(colors[index]); 
     
     meshVertices.push_back(v);
     meshColors.push_back(colors[index]);
     
     meshVertices.push_back(v - pp);
     meshColors.push_back(colors[index]);
     
     meshVertices.push_back(v - pa);
     meshColors.push_back(c);
     
     if (meshVertices.size() >5) {
     for (int i = meshVertices.size()-4; i<meshVertices.size(); i++) {
     pushQuadIndices(i);
     }
     }
     //*/
    
}
//--------------------------------------------------------------
void ofxFatLine::pushQuadIndices(int i1, int i2, int i3, int i4){
    pushTriangleIndices(i1, i3, i4);
    pushTriangleIndices(i1, i2, i4);
}
//--------------------------------------------------------------
void ofxFatLine::pushQuadIndices(int index){
    pushTriangleIndices(index -6, index -5, index);
    pushTriangleIndices(index -6, index -1, index);
}
//--------------------------------------------------------------
void ofxFatLine::pushTriangleIndices(int i1, int i2, int i3){
    meshIndices.push_back((ofIndexType)i1);
    meshIndices.push_back((ofIndexType)i2);
    meshIndices.push_back((ofIndexType)i3);
}
//--------------------------------------------------------------
void ofxFatLine::updateMeshIndices(){
    for (int i = 0; i < meshVertices.size()-5; i+=5) {
        for (int j = 0; j <4; j++) {
            mesh.addIndex(j+i);
            mesh.addIndex(j+i+1);
            mesh.addIndex(j+i+5);
            mesh.addIndex(j+i+1);
            mesh.addIndex(j+i+5);
            mesh.addIndex(j+i+6);
        }   
    }
}
//--------------------------------------------------------------
void ofxFatLine::updateJoint(int index, bool bFlip){
    int l = meshVertices.size()-1;
    if (joint == OFX_FATLINE_JOINT_MITER) {
//        cout << "update joint miter" << endl;
        
    }else if (joint == OFX_FATLINE_JOINT_BEVEL){
//        cout << "update joint bevel" << endl;
        if (bFlip) {
            pushTriangleIndices(l -1, l-2, l-6);
            pushQuadIndices(l -6, l-5, l-1, l);
        }else{
            pushTriangleIndices(l -3, l-2, l-5);
            pushQuadIndices(l -6, l-5, l-4, l-3);        
        }
    }else if (joint == OFX_FATLINE_JOINT_ROUND){
//        cout << "update joint round" << endl;        
        
    }
    
}
//--------------------------------------------------------------
void ofxFatLine::updateCap(glm::vec3 p1, glm::vec3 p2, int index){
    glm::vec3 p = glm::normalize(glm::cross((p1 - p2), glm::vec3(0, 0, 1)));
    bool flip = false;
    glm::vec3 dir = glm::normalize(p2-p1);
    if (cap == OFX_FATLINE_CAP_SQUARE) {
        p2 =  dir * weights[index]*0.5f;
    }
    
    if (getVertices().size()>1) {
        if (index==0 || index == getVertices().size()-1) {
            midVectors.push_back(p);
            if (index != 0) {
                flip = sameSideOfLine(p, flippepMidVectors.back(), p1, p2);
                if (flip) {
                    p*=-1;
                }
            }
            flippepMidVectors.push_back(p*-1);
            
            pushNewAnchors(p2, p, colors[index], weights[index], feathering, true);
            pushNewAnchor(p2, colors[index]);
            p *=-1;
            pushNewAnchors(p2, p, colors[index], weights[index], feathering, false);
            if (index != 0) {
                if (meshVertices.size() >5) {
                    for (int i = meshVertices.size()-4; i<meshVertices.size(); i++) {
                        pushQuadIndices(i);
                    }
                }
            }
        }    
    }/*
      else{
      flip = !sameSideOfLine(p, flippepMidVectors.back(), p1, p2);
      pushNewVertex(p2, p, p, p, index, 1, flip);    
      }//*/
}
//--------------------------------------------------------------
void ofxFatLine::update(){
    updateMesh();    
}
//--------------------------------------------------------------
void ofxFatLine::draw(){
    mesh.draw();
}
//--------------------------------------------------------------
void ofxFatLine::drawDebug(){
    ofPushStyle();
    //ofSetColor(255, 0,127);
    ofDrawCircle(getVertices()[0], 5);
    //ofSetColor(255, 0,0);
    //ofSetLineWidth(3);
    for (int i = 1; i<getVertices().size(); i++) {
        ofDrawLine(getVertices()[i-1], getVertices()[i]);
    }
    
    /*ofMesh m(mesh);
    m.disableColors();
    ofSetLineWidth(1);
    ofSetColor(0);
    m.drawWireframe();*/
    
    ofSetColor(0);
    /*for (int i =0; i < meshVertices.size(); i++) {
        ofDrawBitmapStringHighlight(ofToString(i), meshVertices[i]);
    }*/
    
    ofPopStyle();
    
}
//--------------------------------------------------------------
void ofxFatLine::addColor(const ofFloatColor &c){
    colors.push_back(c);
}
//--------------------------------------------------------------
void ofxFatLine::addColors(const vector<ofFloatColor> &c){
	colors.insert( colors.end(), c.begin(), c.end());
}

//--------------------------------------------------------------
void ofxFatLine::addWeight(const double &w){
    weights.push_back(w);
}
//--------------------------------------------------------------
void ofxFatLine::addWeights(const vector<double> &w){
    weights.insert(weights.end(), w.begin(), w.end());
}

//--------------------------------------------------------------


void ofxFatLine::polylineRange(int from, int to, bool approx){
    if (from > 0) {
        from -= 1;
    }
    
//    bJointFirst = from !=0;
//    bJointLast = to != (getVertices().size()-1);
//    bNoCapFirst = bNoCapFirst || bJointFirst;
//    bNoCapLast = bNoCapLast || bJointLast;
    
    if (approx) {
        polylineApprox(from, to);
    } else {
        polylineExact(from, to);
    }
}

void ofxFatLine::polylineApprox(int from, int to){
    if (to - from + 1 < 2) {
        return;
    }
    
    bool joinFirst =  from !=0;
    bool joinLast = to != (getVertices().size()-1);
    bool capFirst = !(bNoCapFirst || joinFirst);
    bool capLast =  !(bNoCapLast || joinLast);
    
//    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    
    auto poly_step = [this](int i, glm::vec3 pp, float ww, ofColor cc) {
        float t = 0, r = 0;
        DetermineTr(ww, t, r, 1.0f/*opt.worldToScreenRatio*/);
//        if (opt.feather && !opt.noFeatherAtCore) {
//            r *= opt.feathering;
//        }
        float rr = (t + r) / r;
        glm::vec3 V = getVertices()[i] - getVertices()[i - 1];
        V = glm::normalize(glm::vec3(-V.y, V.x, 1));
        V *= (t + r);
        
        meshVertices.push_back(pp - V);
        meshColors.push_back(cc);
        meshVertices.push_back(pp + V);
        meshColors.push_back(cc);
        
//        vcore.Push(pp - V, cc, rr);
//        vcore.Push(pp + V, cc, 0);
    };
    
    for (int i = from + 1; i < to; i++) {
        poly_step(i, getVertices()[i], bUseGlobalWidth ? globalWidth : getWeight(i), getColor(i));
    }
    glm::vec3 P_las, P_fir;
    ofColor C_las , C_fir;
    float W_las, W_fir = 0;
    polyPointInter(P_las, C_las, W_las, to - 1, 0.5f);
    poly_step(to, P_las, W_las, C_las);
    
    //StAnchor SA = new StAnchor();
//               {
//                   polyPointInter(P_fir, C_fir, W_fir, from, joinFirst ? 0.5f : 0.0f);
//                   SA.P[0] = P_fir;
//                   SA.P[1] = P[from + 1];
//                   SA.C[0] = C_fir;
//                   SA.C[1] = color(from + 1);
//                   SA.W[0] = W_fir;
//                   SA.W[1] = weight(from + 1);
//                   Segment(SA, opt, capFirst, false, true);
//               }
//               if (!joinLast) {
//                   SA.P[0] = P_las;
//                   SA.P[1] = P[to];
//                   SA.C[0] = C_las;
//                   SA.C[1] = color(to);
//                   SA.W[0] = W_las;
//                   SA.W[1] = weight(to);
//                   Segment(SA, opt, false, capLast, true);
//               }
//
//               inopt.holder.Push(vcore);
//               inopt.holder.Push(SA.vah);
    
}

void ofxFatLine::polylineExact(int from, int to){
    bool joinFirst = false;
    glm::vec3 mid_l, mid_n; //the last and the next mid point
    ofColor c_l, c_n;
    float w_l = 0, w_n = 0;
    
    //init for the first anchor
    polyPointInter(mid_l, c_l, w_l, from, joinFirst ? 0.5f : 0);
    
//    stAnchor SA = new stAnchor();
//    if (to - from + 1 == 2) {
//        SA.P[0] = P[from];
//        SA.P[1] = P[from + 1];
//        SA.C[0] = color(from);
//        SA.C[1] = color(from + 1);
//        SA.W[0] = weight(from);
//        SA.W[1] = weight(from + 1);
//        Segment(SA, capFirst, capLast, true);
//    } else {
//        for (int i = from + 1; i < to; i++) {
//            if (i == to - 1 && !joinLast) {
//                polyPointInter(mid_n, c_n, w_n, i, 1.0f);
//            } else {
//                polyPointInter(mid_n, c_n, w_n, i, 0.5f);
//            }
//
//            SA.P[0] = mid_l;
//            SA.C[0] = c_l;
//            SA.W[0] = w_l;
//            SA.P[2] = mid_n;
//            SA.C[2] = c_n;
//            SA.W[2] = w_n;
//
//            SA.P[1] = getVertices()[i];
//            SA.C[1] = getColor(i);
//            SA.W[1] = getWeight(i);
//
//            Anchor(SA, opt, (i == 1) && capFirst, (i == to - 1) && capLast);
//
//            mid_l = mid_n;
//            c_l = c_n;
//            w_l = w_n;
//        }
//    }
}

void ofxFatLine::polyPointInter(glm::vec3 &p, ofColor &c, float &w, int at, float t)
{
//    Color color(int I) {
//        return C[inopt != null && inopt.constColor ? 0 : I];
//    }
//    float weight(int I) {
//        return W[inopt != null && inopt.constWeight ? 0 : I];
//    }

    if (t == 0.0) {
        p = getVertices()[at];
        c = getColor(at);
        w = bUseGlobalWidth ? globalWidth : getWeight(at);
    } else if (t == 1.0) {
        p = getVertices()[at + 1];
        c = getColor(at + 1);
        w = bUseGlobalWidth ? globalWidth : getWeight(at + 1);
    } else {
        p = (getVertices()[at] + getVertices()[at + 1]) * t;
//        c = ofLerp Gradient.ColorBetween(color(at), color(at + 1), t);
        c = getColor(at).lerp(getColor(at + 1), t);
        w = bUseGlobalWidth ? globalWidth : ((getWeight(at) + getWeight(at + 1)) * t);
    }
}
