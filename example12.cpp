#include "candle.hpp"

#include <iostream>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>



using namespace std;

bool readAndDisplayRegisters(mab::Candle& candle, uint16_t id);
bool BothreadAndDisplayRegisters(mab::Candle& candle, uint16_t id1, uint16_t id2,int index);
bool Print_both_enc_vals(mab::Candle& candle, int index);
void error();
int NUM_OF_SAMPLES_EXP = 500;
int NUM_OF_SAMPLES_OFFSET = 20;
int ERROR_CODE = 666;
float get_output_encoder_offset(mab::Candle& candle, uint16_t id,int num_of_sampels); 

void write_csv(const std::string& filename, const std::vector<std::string>& vec);

float calc_yaw_offset = 0;
float calc_pitch_offset = 0;

uint16_t id_yaw =0;
uint16_t id_pitch =0;
vector<string> str_vector_exp_data;

// #define DEBUG

int main()
{
	mab::Candle candle(mab::CAN_BAUD_1M, true);
	auto ids = candle.ping();
	/* add the first MD80 to the list */
	candle.addMd80(ids[0]); //204 - YAW
	candle.addMd80(ids[1]);	//205 - PITCH

	id_yaw = ids[0];
	id_pitch = ids[1];
	
	cout << std::endl;

	// time_t now1 = time(0);
	// time_t now2 = time(0);

	//Offset measurements for zero alignment on main enc and out-put enc
	calc_yaw_offset = get_output_encoder_offset(candle,id_yaw,NUM_OF_SAMPLES_OFFSET);
	calc_pitch_offset = get_output_encoder_offset(candle,id_pitch,NUM_OF_SAMPLES_OFFSET);

	string exp_titel = "Index , main 204 , main 205 , Yaw , Pitch" ;
	cout << exp_titel <<endl;
	str_vector_exp_data.push_back(exp_titel);
	
	for (int i=0; i<NUM_OF_SAMPLES_EXP; i++)
	{
		Print_both_enc_vals(candle, i+1);

		//BothreadAndDisplayRegisters(candle, ids[1],ids[1], i+1);
		// now1 = time(0);
		// // Lets first read some registers NOTE: registers cannot be accessed after candle.begin();
		// std::cout << ids[0] << now1 << std::endl;
		// if (!readAndDisplayRegisters(candle, ids[0]))
		// 	std::cout << "Error while reading registers!" << std::endl;
		// std::cout << std::endl;

		// now2 = time(0);
		// std::cout << ids[1] << now2 << std::endl;
		// if (!readAndDisplayRegisters(candle, ids[1]))
		// 	std::cout << "Error while reading registers!" << std::endl;
		// std::cout << std::endl;

		usleep(100*1000);
	}

	write_csv("output_vec.csv", str_vector_exp_data);

	//cout<<str_vector_exp_data<<endl;
	
	return EXIT_SUCCESS;
}


void write_csv(const std::string& filename, const std::vector<std::string>& vec)
 {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file " << filename << " for writing." << std::endl;
        return;
    }
    for (const auto& str : vec) {
        file << str << "\n";
    }
    file.close();
}


bool Print_both_enc_vals(mab::Candle& candle, int index)
{
	/* get the reference to the regR struct that holds fields of registers */
	mab::regRead_st& reg_204_yaw = candle.getMd80FromList(id_yaw).getReadReg();
	mab::regRead_st& reg_205_pitch = candle.getMd80FromList(id_pitch).getReadReg();
	
	if (!candle.readMd80Register(id_yaw, mab::Md80Reg_E::mainEncoderPosition,reg_204_yaw.RO.mainEncoderPosition)) return false;
	if (!candle.readMd80Register(id_pitch, mab::Md80Reg_E::mainEncoderPosition,reg_205_pitch.RO.mainEncoderPosition)) return false;
	
	if (!candle.readMd80Register(id_yaw, mab::Md80Reg_E::outputEncoderPosition, reg_204_yaw.RO.outputEncoderPosition)) return false;
	if (!candle.readMd80Register(id_pitch, mab::Md80Reg_E::outputEncoderPosition, reg_205_pitch.RO.outputEncoderPosition)) return false;

	
	float mainEnc_204 = reg_204_yaw.RO.mainEncoderPosition;
	float mainEnc_205 = reg_205_pitch.RO.mainEncoderPosition;
	
	float opEnc_yaw = reg_204_yaw.RO.outputEncoderPosition		- calc_yaw_offset;
	float opEnc_pitch = reg_205_pitch.RO.outputEncoderPosition	- calc_pitch_offset;

	string str_samples_line = to_string(index) +" , "+ to_string(mainEnc_204)+ " , "+ to_string(mainEnc_205) + " , "+ to_string(opEnc_yaw)+ " , "+ to_string(opEnc_pitch) + " ";

	cout <<str_samples_line<<endl;
	str_vector_exp_data.push_back(str_samples_line);
	//std::cout << "INDEX , main 204 , main 205 , YAW , PITCH"<< std::endl;
	// cout << index <<" , "<< mainEnc_204 << " , " << mainEnc_205 << " , " << opEnc_yaw<< " , " <<opEnc_pitch<< std::endl;

	return true;
}

bool BothreadAndDisplayRegisters(mab::Candle& candle, uint16_t id1, uint16_t id2, int index)
{
	/* get the reference to the regR struct that holds fields of registers */
	mab::regRead_st& reg1 = candle.getMd80FromList(id1).getReadReg();
	mab::regRead_st& reg2 = candle.getMd80FromList(id2).getReadReg();

	if (!candle.readMd80Register(id1, mab::Md80Reg_E::mainEncoderPosition,reg1.RO.mainEncoderPosition)) return false;
	if (!candle.readMd80Register(id2, mab::Md80Reg_E::outputEncoderPosition, reg2.RO.outputEncoderPosition)) return false;
	
	cout << index <<" , "<< reg1.RO.mainEncoderPosition << " , " << reg2.RO.mainEncoderPosition << " , " << reg1.RO.outputEncoderPosition<< " , " << reg2.RO.outputEncoderPosition<< std::endl;

	return true;
}

float get_output_encoder_offset(mab::Candle& candle, uint16_t id ,int num_of_sampels)
{
	float encoder_offset =0.00;
	float offset_sum=0.00;

	for (int i=0; i< num_of_sampels; i++)
	{
		mab::regRead_st& reg = candle.getMd80FromList(id).getReadReg();
		if (!candle.readMd80Register(id, mab::Md80Reg_E::outputEncoderPosition, reg.RO.outputEncoderPosition)) return ERROR_CODE;
		offset_sum += reg.RO.outputEncoderPosition;
		#ifdef DEBUG
		std::cout << "this is offset_sum: " <<offset_sum << std::endl;
		#endif
	}

	encoder_offset = offset_sum/num_of_sampels;
	
	#ifdef DEBUG
		std::cout << "this is encoder_offset: " <<encoder_offset << std::endl;
	#endif

	return encoder_offset;

}

bool readAndDisplayRegisters(mab::Candle& candle, uint16_t id)
{
	/* get the reference to the regR struct that holds fields of registers */
	mab::regRead_st& reg = candle.getMd80FromList(id).getReadReg();

	if (!candle.readMd80Register(id, mab::Md80Reg_E::mainEncoderPosition,reg.RO.mainEncoderPosition)) return false;
	if (!candle.readMd80Register(id, mab::Md80Reg_E::outputEncoderPosition, reg.RO.outputEncoderPosition)) return false;

	
	std::cout << id <<"'s Main En Pos: " << reg.RO.mainEncoderPosition << " " << "// The OP Enc Pos: " << reg.RO.outputEncoderPosition<< std::endl;

	return true;
}




void error()
{
	std::cout << "Error while writing register!" << std::endl;
}





// #include "candle.hpp"

// bool readAndDisplayRegisters(mab::Candle& candle, uint16_t id);
// void error();

// int main()
// {
// 	mab::Candle candle(mab::CAN_BAUD_1M, true);
// 	auto ids = candle.ping();

// 	/* add the first MD80 to the list */
// 	candle.addMd80(ids[0]);

// 	std::cout << std::endl;

// 	// Lets first read some registers NOTE: registers cannot be accessed after candle.begin();
// 	if (!readAndDisplayRegisters(candle, ids[0]))
// 		std::cout << "Error while reading registers!" << std::endl;

// 	std::cout << std::endl;

// 	// Then change some parameters
// 	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorName, "EXAMPLE12")) error();
// 	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorImpPidKp, 1.0f)) error();
// 	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorImpPidKd, 0.01f)) error();
// 	if (!candle.writeMd80Register(ids[0], mab::Md80Reg_E::motorImpPidOutMax, 1.0f)) error();

// 	// And read one more time NOTE: these settings are not saved!
// 	if (!readAndDisplayRegisters(candle, ids[0]))
// 		std::cout << "Error while reading registers!" << std::endl;

// 	return EXIT_SUCCESS;
// }

// bool readAndDisplayRegisters(mab::Candle& candle, uint16_t id)
// {
// 	/* get the reference to the regR struct that holds fields of registers */
// 	mab::regRead_st& reg = candle.getMd80FromList(id).getReadReg();

// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::canId, reg.RW.canId)) return false;
// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorName, reg.RW.motorName)) return false;
// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorImpPidKp, reg.RW.impedancePdGains.kp)) return false;
// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorImpPidKd, reg.RW.impedancePdGains.kd)) return false;
// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::motorImpPidOutMax, reg.RW.impedancePdGains.outMax)) return false;

// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::mainEncoderPosition,reg.RO.mainEncoderPosition)) return false;
	
// 	if (!candle.readMd80Register(id, mab::Md80Reg_E::outputEncoderPosition, reg.RO.outputEncoderPosition)) return false;

// 	std::cout << "Drive ID: " << unsigned(reg.RW.canId) << std::endl;
// 	std::cout << "Motor name: " << std::string(reg.RW.motorName) << std::endl;
// 	std::cout << "Impedance mode Kp gain : " << reg.RW.impedancePdGains.kp << " Nm/rad" << std::endl;
// 	std::cout << "Impedance mode Kd gain : " << reg.RW.impedancePdGains.kd << " Nm*s/rad" << std::endl;
// 	std::cout << "Impedance mode max out : " << reg.RW.impedancePdGains.outMax << " Nm" << std::endl;

// 	std::cout << "RO.mainEncoderPosition : " << reg.RO.mainEncoderPosition << " rad - MAIN"<< std::endl;
// 	std::cout << "RO.outputEncoderPosition : " << reg.RO.outputEncoderPosition<< " mode - OUT"<<std::endl;

// 	return true;
// }

// void error()
// {
// 	std::cout << "Error while writing register!" << std::endl;
// }






