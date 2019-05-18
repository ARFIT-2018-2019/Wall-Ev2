#include "FonctionsMath.h"
#include "Asservissement.h"
class Task {
	public : 
		virtual bool execute() = 0;
};

class MoveTask : public Task{
	public :
		MoveTask(Position positionCible, int timeOut){
		  this->positionCible = positionCible;
		};

		bool execute() override {
		  //TournerDe(positionRobot->angle - position->angl, timeOut);
      int x = positionRobot.x;
      Serial.print("have to go ");
      Serial.println(x);
      TranslaterDe(abs(positionRobot.x - positionCible.x), timeOut);
      return true;
		};
    
	private :
		Position positionCible;
		int timeOut;
};
