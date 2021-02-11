#pragma once
#include "ofMain.h"
#include "sincosineLut.h"

//--------------------------------------------------------------
inline bool lineSegmentsIntersect(const glm::vec2 & l1a, const glm::vec2 & l1b, const glm::vec2 & l2a, const glm::vec2 & l2b){
	if(l1a == l2a || l1a == l2b || l1b == l2a || l1b == l2b){
		return true;
	}

	float m1, m2;//slopes
	bool infM1 = false, infM2 = false;
	bool equalSlope = false;
	if (l1a.x == l1b.x) {
		infM1 =true;
	}
	if (l2a.x == l2b.x) { //the slope is equal to infinite, or a division by 0 is created. 
		infM2 =true;
	}
	if (infM1 && infM2) {
		equalSlope = true;
	}else{
		if(!infM1){//check so we dont divide by 0
			m1 = (l1b.y - l1a.y)/(l1b.x - l1a.x);//slope line 1
		}
		if (!infM2) {
			m2 = (l2b.y - l2a.y)/(l2b.x - l2a.x);//slope line 2		
		}
		
		if (!infM1 && !infM2) {
			if (m1 == m2) {
				equalSlope = true;
			}
		}
	}

    return equalSlope;
	
}
//--------------------------------------------------------------
inline double sideOfLine(const glm::vec2& v, const glm::vec2& a, const glm::vec2& b){
    glm::vec2 dir = glm::normalize(b-a);
    dir = glm::vec2(-dir.y, dir.x);
    dir = glm::normalize(dir);
    return glm::dot(glm::normalize(v), dir);
}

static double signed_area(const glm::vec2& P1, const glm::vec2& P2, const glm::vec2& P3)
{
    return (P2.x-P1.x)*(P3.y-P1.y) - (P3.x-P1.x)*(P2.y-P1.y);
}

//--------------------------------------------------------------
static inline bool sameSideOfLine( const glm::vec2& V, const glm::vec2& ref, const glm::vec2& a, const glm::vec2& b){
//	double sign1 = sideOfLine(V*100, a, b);
//    double sign2 = sideOfLine(ref*100, a, b);
//    return !( (sign1>=0) ^ (sign2>=0));
    return true;
    double sign1 = signed_area( a+ref,a,b);
    double sign2 = signed_area( a+V,  a,b);
    return !( (sign1>=0) != (sign2>=0));
//    {
//        V.opposite();
//    }
    
}
//--------------------------------------------------------------
static inline glm::vec3 getMidVector(glm::vec3 &a, glm::vec3 &b){
    return glm::normalize(glm::normalize(a) + glm::normalize(b));
}

static void DetermineTr(float w, float &t, float &R, float scale)
{
    //efficiency: can cache one set of w,t,R values
    // i.e. when a polyline is of uniform thickness, the same w is passed in repeatedly
    w *= scale;
    float f = w - (float)floor(w);
    // resolution dependent
    if (w >= 0.0 && w < 1.0) {
        t = 0.05f;
        R = 0.768f;
    } else if (w >= 1.0 && w < 2.0) {
        t = 0.05f + f * 0.33f;
        R = 0.768f + 0.312f * f;
    } else if (w >= 2.0 && w < 3.0) {
        t = 0.38f + f * 0.58f;
        R = 1.08f;
    } else if (w >= 3.0 && w < 4.0) {
        t = 0.96f + f * 0.48f;
        R = 1.08f;
    } else if (w >= 4.0 && w < 5.0) {
        t = 1.44f + f * 0.46f;
        R = 1.08f;
    } else if (w >= 5.0 && w < 6.0) {
        t = 1.9f + f * 0.6f;
        R = 1.08f;
    } else if (w >= 6.0) {
        t = 2.5f + (w - 6.0f) * 0.50f;
        R = 1.08f;
    }
    t /= scale;
    R /= scale;
}

enum ofxFatLineJointType{
    OFX_FATLINE_JOINT_MITER, //Default
    OFX_FATLINE_JOINT_BEVEL,
    OFX_FATLINE_JOINT_ROUND
};
enum ofxFatLineCapType{
    OFX_FATLINE_CAP_BUTT = 0, //Default
    OFX_FATLINE_CAP_ROUND = 1,
    OFX_FATLINE_CAP_SQUARE = 2,
    OFX_FATLINE_CAP_RECT = 3,
    OFX_FATLINE_CAP_BOTH = 0, //Default
    OFX_FATLINE_CAP_FIRST = 10,
    OFX_FATLINE_CAP_LAST = 20,
    OFX_FATLINE_CAP_NONE = 30
};


class ofxFatLine : public ofPolyline{
public:
    ofxFatLine();
    ofxFatLine(const vector<glm::vec3> &P,const vector<ofFloatColor> &C, const vector<double> &W, bool triangulation = false );

    void add(const glm::vec3 &thePoint, const ofFloatColor &theColor, const double &theWeight);
    void add(const vector<glm::vec3> &thePoints, const vector<ofFloatColor> &theColors, const vector<double> &theWeights);
	
	void setFromPolyline(ofPolyline & poly);
    
    void enableFeathering(bool e = true){bFeather = e;}
    void toggleFeathering(){enableFeathering(!bFeather);}
    
    void setFeather(double f){feathering = f; update();}
    double getFeather(){return feathering;}
    
    void setJointType(ofxFatLineJointType j){joint = j;}
    ofxFatLineJointType getJointType(){return joint;}
    
    void setCapType(ofxFatLineCapType c ){cap = c;}
    ofxFatLineCapType getCapType(){return cap;}
    
    void enableTriangulation(bool e = true){bTriangulation = e; }
    void toggleTriangulation(){enableTriangulation(!bTriangulation);}

    void enableFeatherAtCore(bool e = true){bFeatherAtCore =e; }
    void toggleFeatherAtCore(){enableFeatherAtCore(!bFeatherAtCore);}

    void enableFeatherAtCap(bool e = true){bFeatherAtCap =e; }
    void toggleFeatherAtCap(){enableFeatherAtCap(!bFeatherAtCap);}    
    
	void setGlobalColor(ofColor col);
	void setGlobalColor(ofFloatColor col);
	
	void setGlobalWidth(float w);
	
    void draw();
    void update();
    void updatePoint(int index, glm::vec3 p);
    void updateWeight(int index, float w);
    float getWeight(int index);
    void updateColor(int index, ofFloatColor& c);
    ofColor getColor(int index);
    ofMesh &getMesh(){return mesh;}
    void drawDebug();
    void printDebug();
    
    void polylineRange(int from, int to, bool aprox);
    void polylineApprox(int from, int to);
    void polylineExact(int from, int to);
    void polyPointInter(glm::vec3 &p, ofColor &c, float &w, int at, float t);
    void addColor(const ofFloatColor &c);
    void addColors(const vector<ofFloatColor> &c);
    void addWeight(const double &w);
    void addWeights(const vector<double> &w);

protected:

    void pushQuadIndices(int index);
    void pushQuadIndices(int i1, int i2, int i3, int i4);
    void pushNewVertex(glm::vec3 v, glm::vec3 p, glm::vec3 r1, glm::vec3 r2, float maxLength, int index, float cos, bool bFlipped = false);
    void pushNewAnchors(glm::vec3 v, glm::vec3 dir, ofFloatColor color, float l1, float l2, bool bInv);
    void pushNewAnchor(glm::vec3 a, ofFloatColor c);
    void pushTriangleIndices(int i1, int i2, int i3);
    void updateCap(glm::vec3 p1, glm::vec3 p2, int index);
    void updateMesh();
    void updateMeshIndices();
    void updateJoint(int index, bool bFlip);
    void updateVertex(int index);

    ofMesh mesh, capJointMesh;
    
    vector<ofFloatColor> colors;
    vector<double> weights;
    vector<glm::vec3> midVectors;
    vector<glm::vec3> flippepMidVectors;
    vector<glm::vec3> crossedVectors;
    vector<glm::vec3> meshVertices;
    vector<ofIndexType> meshIndices;
    vector<ofFloatColor> meshColors;
    vector<glm::vec3>jointMeshVertices;
    vector<ofIndexType>jointMeshIndices;
    vector<ofFloatColor>jointMeshColors;
	
	
    //OPT
	bool bFeather;
    float feathering;
	bool bFeatherAtCap;
	bool bFeatherAtCore;
    float worldToScreenRatio;
    bool bTriangulation;
    
    ofxFatLineJointType joint;
    ofxFatLineCapType cap;

    //Inopt
    bool bUseGlobalColor;
    bool bUseGlobalWidth;
    bool bNoCapFirst;
    bool bNoCapLast;
    bool bJointFirst;
    bool bJointLast;
    
    
    //No use
    ofFloatColor globalColor;
    float globalWidth;
};
