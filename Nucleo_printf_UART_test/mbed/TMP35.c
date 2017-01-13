#include "TMP35.h"
float temperature(float voltage)
{
	return (voltage*3300 - 500)/10;
}
