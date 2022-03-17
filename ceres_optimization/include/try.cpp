#include<iostream>
using namespace std;

class Thing {
public:
    int num = 1;
    Thing(int num) { 
        //this->num = num; 
    }
};

class Stuff {
public:
    Thing thing=Thing(0);  // an instance of thing is declared here but it cannot construct it
    Stuff(Thing thing) {
        this->thing = thing;
    }
};

int main() {
    Thing thing = Thing(5);
    Stuff stuff = Stuff(thing);
    cout << thing.num ;
}
