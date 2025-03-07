#include <vector>
#include <iostream>
#include <string>
#include <cmath>  // Llibreria matemàtica

using namespace std; // hola Luis Florit

// aquesta funció anomenada calculaRumb, ens calcula l'angle de rumb
// per anar del punt "pos1" al punt "pos2"
float calculaRumb(const vector<float>& pos1, const vector<float>& pos2 ) {
    float ang = atan( (pos2[0] - pos1[0])/ (pos2[1] - pos1[1]) );
    ang = ang*360/2/M_PI;
    return ang;
}


int main() {
    vector<float> posActual = {41.263790, 1.957213}; // posicio del vaixell
    vector<float> pos2 = {41.255684, 1.967816}; // posicio de destí
    vector<float> pos3 = {41.258898, 1.930067}; // un altre punt de destí
    
        
    float ang2 = calculaRumb(posActual, pos2); // calculem l'angle que 
                                            // necessitem per anar al punt 2
    float ang3 = calculaRumb(pos2, pos3);// calculem l'angle que necessitem
                                    // per anar al punt 3

    // imprimim per pantalla els angles que hem trobat:
    cout << "Angle del rumb desitjat: " << ang2 << endl;
    cout << "Segon angle: " << ang3 << endl;

    return 0;
}