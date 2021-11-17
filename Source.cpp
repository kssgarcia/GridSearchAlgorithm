#include <iostream>
#include <math.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <stdio.h>

#define PI 3.14159265

class Actuator
{
private:
	// Data 
	float* force;
	float* angleD;
	// Actuator details
	float highF;
	float maxForce;
	float stroke;
	float minLength;
	// constructor members 
	float angle;
	int direction;
	float airLoad;
	float weight;
	float step;
	int n;
	// Class members
	float A1Coord[2] = { -0.270, -0.722 };
	float A2Coord[2] = { 0, 0 };
	float angleA2 = 1.019;
	float distA2 = 0.38160;
	float BCoord[2] = { -0.14142, 0.14142 };
	float ActuatorPos[2] = { 0, 0 };
	float ActuatorAngle = 0;
	float ActuatorLen = 0;
	float lenInit = 1;
	float strokeS = 0;
	float tol = 0;

	float num = 0;

public:
	Actuator()
	{
		highF = 0.2;
		maxForce = 9000;
		stroke = 0.450;
		minLength = 0.2;
		angle = 135;
		direction = -1;
		airLoad = 0.4;
		weight = -4561.65;
		step = 0.1;
		n = ((angle - 45) / step) - 100;
		force = new float[n + 1];
		angleD = new float[n + 1];
	}

	Actuator(float highF, float maxForce, float stroke, float minLength, float angle, int direction, float airLoad, float weight, float step, float tol = 0.03)
		: highF(highF), maxForce(maxForce), stroke(stroke), minLength(minLength), angle(angle), direction(direction), airLoad(airLoad), weight(weight), step(step), tol(tol)
	{
		n = (angle - 45) / step;
		force = new float[n + 1];
		angleD = new float[n + 1];
	}

	~Actuator()
	{
		delete[] force;
		delete[] angleD;
	}

	float* Solve()
	{
		float airForce = 0;
		int e = 0;
		lenInit = 0;
		ActuatorAngle = 0;
		ActuatorLen = 0;
		for (float i = angle, angleRad = 0; i <= 135.0 && i >= 45.0; i += direction * step)
		{
			angleRad = i * PI / 180;
			// Calculate the force of the air on the center of mass
			airForce = airLoad * 4.41 * 4.128 * sin(abs(angleRad - 1.5708)) * 1000;
			// Calculate coordinate of the point B
			BCoord[0] = cos(angleRad) * highF;
			BCoord[1] = sin(angleRad) * highF; 
			// Calculate coordinate of the point A2
			A2Coord[0] = cos((angleRad + angleA2)) * distA2;
			A2Coord[1] = sin((angleRad + angleA2)) * distA2;
			// Calculate the components of the actuator 
			ActuatorPos[0] = A2Coord[0] - A1Coord[0];
			ActuatorPos[1] = A2Coord[1] - A1Coord[1];
			// Calculate the angle of the actuator
			ActuatorAngle = (atan(ActuatorPos[0] / ActuatorPos[1]) - 1.5708);
			// Calculate the actuator length
			ActuatorLen = sqrt(pow(ActuatorPos[0], 2) + pow(ActuatorPos[1], 2));
			// Validate the actuator
			if (e == 0) { lenInit = ActuatorLen; }
			if (!Compare(lenInit, stroke + minLength, tol, tol)) { break; }
			if (e == n) { strokeS = ActuatorLen - lenInit; }
			// Calculate the force and push into the 
			force[e] = abs((-airForce * BCoord[1] + weight * BCoord[0]) / (A2Coord[0] * sin(ActuatorAngle) + A2Coord[1] * cos(ActuatorAngle)));
			angleD[e] = -(i - 90);
			e++;
			num++;
		}
		return force;
	}

	void SearchPosition(float* xRangeA1, float* yRangeA1, float stepRangeA1, float* xRangeA2, float* yRangeA2, float stepRangeA2)
	{
		int len = (((xRangeA2[1] - xRangeA2[0]) / stepRangeA2) + 1) * (((yRangeA2[1] - yRangeA2[0]) / stepRangeA2) + 1) * (((yRangeA1[1] - yRangeA1[0]) / stepRangeA1) + 1) * (((xRangeA1[1] - xRangeA1[0]) / stepRangeA1) + 1);
		// Searh map array
		std::vector<float> mapXA1;
		std::vector<float> mapYA1;
		std::vector<float> mapA1Color;
		std::vector<float> mapXA2;
		std::vector<float> mapYA2;
		std::vector<float> mapA2Color;
		int count = 0;
		for (float A2Y = yRangeA2[0]; A2Y >= yRangeA2[0] && A2Y <= yRangeA2[1]; A2Y += stepRangeA2)
		{
			for (float A2X = xRangeA2[0]; A2X >= xRangeA2[0] && A2X <= xRangeA2[1]; A2X += stepRangeA2)
			{
				// Calculate the angle of the point A2 and it distance to the center of the coordinate system
				angleA2 = atan(A2X / A2Y);
				distA2 = A2X / sin(angleA2);
				// Iterate about the positon Y of the point A1
				for (A1Coord[1] = yRangeA1[0]; A1Coord[1] < yRangeA1[1] && A1Coord[1] >= yRangeA1[0]; A1Coord[1] += stepRangeA1)
				{
					// Iterate about the positon X of the point A1
					for (A1Coord[0] = xRangeA1[1]; A1Coord[0] <= xRangeA1[1] && A1Coord[0] > xRangeA1[0]; A1Coord[0] -= stepRangeA1)
					{
						// Call the funcion Solve() to calculate the force for this actuator position
						Solve();
						// Check the maximun force the actuator can hold
						if (force[n] < maxForce)
						{
							// Filtrate the data 
							if (Compare(strokeS, stroke, tol, tol) && abs(A1Coord[0]) > 0.1 && Compare(ActuatorLen, stroke * 2 + minLength, tol, 0.02))
							{
								std::cout << count << " | " << lenInit << " | " << strokeS << " | " << force[n] << " | " << A2X  << " | " << A2Y << " | " << A1Coord[0] << " | " << A1Coord[1] << " | " << ActuatorLen << std::endl;
								count++;
								if (count == 16)
								{
									Plot();
								}
								// Set the value's points A1
								mapXA1.push_back(A1Coord[0]);
								mapYA1.push_back(A1Coord[1]);
								mapA1Color.push_back(1);
								// Set the value's points A2
								mapXA2.push_back(-A2X);
								mapYA2.push_back(-A2Y);
								mapA2Color.push_back(1);
							}
						}
					}
				}
			}
		}
		SearchMap(mapXA1, mapYA1, mapA1Color, mapXA2, mapYA2, mapA2Color);
	}

	void SearchMap(std::vector<float> X, std::vector<float> Y, std::vector<float> color, std::vector<float> XA2, std::vector<float> YA2, std::vector<float> colorA2)
	{
		std::ofstream file1;
		file1.open("A2Data.csv"); 
		for (int i = 0; i < XA2.size(); i++)
		{
			file1 << XA2[i] << "," << YA2[i] << "," << colorA2[i] << std::endl;
		}
		file1.close();

		std::ofstream file2;
		file1.open("mapData.csv");
		for (int i = 0; i < X.size(); i++)
		{
			file1 << X[i] << "," << Y[i] << "," << color[i] << std::endl;
		}
		file1.close();

		std::ofstream file3;
		file2.open("mapPlotting.gnu");
		file2 << "set datafile sep ','" << std::endl;
		file2 << "set terminal png size 1200.941 crop" << std::endl;
		file2 << "set title 'Map algorithm'" << std::endl;
		file2 << "set output 'plotMap.png'" << std::endl;
		file2 << "set xlabel 'Y[mm]'" << std::endl;
		file2 << "set ylabel 'X[mm]'" << std::endl;
		file2 << "set xrange [-0.52:0.1]" << std::endl;
		file2 << "set yrange [-1:0.5]" << std::endl;
		file2 << "set palette rgb 33,12,10" << std::endl;
		file2 << "plot 'mapData.csv' u 1:2:3:3 w p lt 7 ps var lc var, 'A2Data.csv' u 1:2:3:3 w p lt 7 ps var lc var" << std::endl;
		file2.close();

		system("gnuplot mapPlotting.gnu");
		system("plotMap.png");
	}

	bool Compare(float& num1, float num2, float tolMin, float tolMax)
	{
		return num2 - tolMin < num1&& num1 < num2 + tolMax ? true : false;
	}

	void Plot()
	{
		std::ofstream file1;
		file1.open("force.csv");
		int x = 0;
		for (int i = 0; i < n; i++)
		{
			file1 << angleD[i] << "," << force[i] << std::endl;
			x++;
		}
		file1.close();

		std::ofstream file2;
		file2.open("plot.gnu");
		file2 << "set datafile sep ','" << std::endl;
		file2 << "set terminal png size 1200.941 crop" << std::endl;
		file2 << "set title 'Actuator force'" << std::endl;
		file2 << "set output 'plot.png'" << std::endl;
		file2 << "set xlabel 'Inclination[°]'" << std::endl;
		file2 << "set ylabel 'Force[N]'" << std::endl;
		file2 << "plot 'force.csv'" << std::endl;
		file2.close();

		system("gnuplot plot.gnu");
		system("plot.png");
	}
};

int main()
{
	// First actuator 
	//Actuator Sol1(8950, 0.450, 0.2, 135, -1, 0.4, -4561.65, 0.1);
	//Sol1.SearchPosition(0.3, 0.75, 0.01, 0.2, 0.5, 0.01);

	Actuator Sol1(0.278, 10000, 0.45, 0.26, 135, -1, 0.4, -4561.65, 0.1, 0.01);

	float xRangeA1[2] = { -0.5, -0.05 };
	float yRangeA1[2] = { -1.0, -0.1 };
	float xRangeA2[2] = { 0.2, 0.5 };
	float yRangeA2[2] = { 0.062, 0.062 };
	Sol1.SearchPosition(xRangeA1, yRangeA1, 0.01, xRangeA2, yRangeA2, 0.01);
}
