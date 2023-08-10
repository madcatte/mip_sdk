
/////////////////////////////////////////////////////////////////////////////
//
// CV7_INS_GQ7_Example.cpp
//
// C++ Example set-up program for CV7-INS with external aiding using a GQ7
//
// This examples is meant to show how to provide external aiding measurements to a CV7-INS.
// For simplicity, this example uses a GQ7 to provide the aiding measurements.
// In practice, you would want to use this example as a starting point
// and switch out the GQ7 measurements for the sensors on your system.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, PARKER MICROSTRAIN SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include <mip/extras/recording_connection.hpp>
#include <mip/platform/serial_connection.hpp>
#include <mip/mip_all.hpp>
#include <array>
#include <math.h>
#include "../example_utils.hpp"

using namespace mip;


////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

//Sensor-to-vehicle frame transformation (Euler Angles)
float gq7_sensor_to_vehicle_transformation_euler[3] = {0.0, 0.0, 0.0};

//GNSS antenna offsets
float gq7_gnss1_antenna_offset_meters[3] = {-0.25, 0.0, 0.0};
float gq7_gnss2_antenna_offset_meters[3] = {0.25, 0.0, 0.0};

//CV7 to GQ7 transform
float cv7_to_gq7_translation[3] = {0, 0, 0};  // x, y, z
float cv7_to_gq7_rotation[3]    = {0, 0, 0};  // roll, pitch, yaw

//Device data stores
data_shared::GpsTimestamp gq7_gnss_gps_time;
data_gnss::FixInfo        gq7_gnss_fix_info;
data_gnss::PosLlh         gq7_gnss_pos_llh;
data_gnss::VelNed         gq7_gnss_vel_ned;

bool gq7_gnss_fix_info_valid = false;

data_shared::GpsTimestamp           gq7_filter_gps_time;
data_filter::Status                 gq7_filter_status;
data_filter::VelocityNed            gq7_filter_velocity_ned;
data_filter::VelocityNedUncertainty gq7_filter_velocity_ned_uncertainty;
data_filter::AttitudeQuaternion     gq7_filter_attitude_quaternion;
data_filter::GnssDualAntennaStatus  gq7_filter_dual_antenna_status;

bool gq7_filter_state_full_nav = false;
bool gq7_filter_dual_antenna_status_fix = false;

data_shared::GpsTimestamp cv7_filter_gps_time;
data_filter::Status       cv7_filter_status;
data_filter::PositionLlh  cv7_filter_position_llh;
data_filter::VelocityNed  cv7_filter_velocity_ned;
data_filter::EulerAngles  cv7_filter_euler_angles;

bool cv7_filter_state_full_nav = false;

////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

double gps_time_to_seconds(const double week_number, const double time_of_week);

void rotate_vector_by_quaternion(const float* vector_in, const float* quaternion, float* vector_out);

int usage(const char* argv0);

void exit_gracefully(const char *message);
bool should_exit();


////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{
    std::unique_ptr<ExampleUtils> gq7_utils = handleCommonArgs(argc, argv, 7);
    std::unique_ptr<mip::DeviceInterface>& gq7_device = gq7_utils->device;

    std::unique_ptr<ExampleUtils> cv7_utils = handleCommonArgs(argc - 3, &argv[3]);
    std::unique_ptr<mip::DeviceInterface>& cv7_device = cv7_utils->device;


    //
    //GQ7 Setup
    //

    printf("Connecting to and configuring GQ7.\n");


    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(commands_base::ping(*gq7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping GQ7!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(commands_base::setIdle(*gq7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set GQ7 to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(commands_3dm::defaultDeviceSettings(*gq7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default GQ7 settings!");


    //
    //Setup GNSS 1 and 2 data format to 2 Hz (decimation of 1)
    //

    std::array<DescriptorRate, 4> gnss_1_descriptors = {{
        { data_shared::DATA_GPS_TIME,   1 },
        { data_gnss::DATA_FIX_INFO,     1 },
        { data_gnss::DATA_POSITION_LLH, 1 },
        { data_gnss::DATA_VELOCITY_NED, 1 },
    }};

    //GNSS1
    if(commands_3dm::writeMessageFormat(*gq7_device, data_gnss::MIP_GNSS1_DATA_DESC_SET, gnss_1_descriptors.size(), gnss_1_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS1 message format from GQ7!");

    //
    //Setup FILTER data format
    //

    uint16_t gq7_filter_base_rate;

    if(commands_3dm::getBaseRate(*gq7_device, data_filter::DESCRIPTOR_SET, &gq7_filter_base_rate) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not get filter base rate format from GQ7!");

    const uint16_t gq7_filter_sample_rate = 100; // Hz
    const uint16_t gq7_filter_decimation = gq7_filter_base_rate / gq7_filter_sample_rate;

    std::array<DescriptorRate, 6> gq7_filter_descriptors = {{
        { data_shared::DATA_GPS_TIME,                 gq7_filter_decimation },
        { data_filter::DATA_FILTER_STATUS,            gq7_filter_decimation },
        { data_filter::DATA_VEL_NED,                  gq7_filter_decimation },
        { data_filter::DATA_VEL_UNCERTAINTY,          gq7_filter_decimation },
        { data_filter::DATA_ATT_QUATERNION,           gq7_filter_decimation },
        { data_filter::DATA_GNSS_DUAL_ANTENNA_STATUS, gq7_filter_decimation },
    }};

    if(commands_3dm::writeMessageFormat(*gq7_device, data_filter::DESCRIPTOR_SET, gq7_filter_descriptors.size(), gq7_filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format on GQ7!");


    //
    //Setup the sensor to vehicle transformation
    //

    if(commands_3dm::writeSensor2VehicleTransformEuler(*gq7_device, gq7_sensor_to_vehicle_transformation_euler[0], gq7_sensor_to_vehicle_transformation_euler[1], gq7_sensor_to_vehicle_transformation_euler[2]) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-to-vehicle transformation on GQ7!");

    //
    //GPIO Config for GQ7
    //
    if(commands_3dm::writeGpioConfig(*gq7_device, 1, mip::commands_3dm::GpioConfig::Feature::PPS,
       mip::commands_3dm::GpioConfig::Behavior::PPS_OUTPUT, mip::commands_3dm::GpioConfig::PinMode::NONE) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set GPIO config on GQ7");

    //
    //Setup the GNSS antenna offsets
    //

    //GNSS1
    if(commands_filter::writeMultiAntennaOffset(*gq7_device, 1, gq7_gnss1_antenna_offset_meters) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS1 antenna offset on GQ7!");

    //GNSS2
    if(commands_filter::writeMultiAntennaOffset(*gq7_device, 2, gq7_gnss2_antenna_offset_meters) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set GNSS2 antenna offset on GQ7!");


    //
    //Setup the filter aiding measurements (GNSS position/velocity and dual antenna [aka gnss heading])
    //

    if(commands_filter::writeAidingMeasurementEnable(*gq7_device, commands_filter::AidingMeasurementEnable::AidingSource::GNSS_POS_VEL, true) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable on GQ7!");

    if(commands_filter::writeAidingMeasurementEnable(*gq7_device, commands_filter::AidingMeasurementEnable::AidingSource::GNSS_HEADING, true) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter aiding measurement enable on GQ7!");


    //
    //Enable the wheeled-vehicle constraint
    //

    if(commands_filter::writeWheeledVehicleConstraintControl(*gq7_device, 1) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter wheeled-vehicle constraint enable on GQ7!");


    //
    //Setup the filter initialization (note: heading alignment is a bitfield!)
    //

    float gq7_filter_init_pos[3] = {0};
    float gq7_filter_init_vel[3] = {0};

    commands_filter::InitializationConfiguration::AlignmentSelector gq7_alignment;
    gq7_alignment = gq7_alignment.DUAL_ANTENNA;

    if(commands_filter::writeInitializationConfiguration(*gq7_device, 0, commands_filter::InitializationConfiguration::InitialConditionSource::AUTO_POS_VEL_ATT, 
       gq7_alignment, 0.0, 0.0, 0.0, gq7_filter_init_pos, gq7_filter_init_vel, commands_filter::FilterReferenceFrame::LLH) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter initialization configuration on GQ7!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(commands_filter::reset(*gq7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter on GQ7!");

    //
    // Register data callbacks
    //

    //GNSS Data
    DispatchHandler gq7_gnss_data_handlers[4];

    gq7_device->registerExtractor(gq7_gnss_data_handlers[0], &gq7_gnss_gps_time, data_gnss::MIP_GNSS1_DATA_DESC_SET);
    gq7_device->registerExtractor(gq7_gnss_data_handlers[1], &gq7_gnss_fix_info, data_gnss::MIP_GNSS1_DATA_DESC_SET);
    gq7_device->registerExtractor(gq7_gnss_data_handlers[2], &gq7_gnss_pos_llh, data_gnss::MIP_GNSS1_DATA_DESC_SET);
    gq7_device->registerExtractor(gq7_gnss_data_handlers[3], &gq7_gnss_vel_ned, data_gnss::MIP_GNSS1_DATA_DESC_SET);

    //Filter Data
    DispatchHandler gq7_filter_data_handlers[6];

    gq7_device->registerExtractor(gq7_filter_data_handlers[0], &gq7_filter_gps_time, data_filter::DESCRIPTOR_SET);
    gq7_device->registerExtractor(gq7_filter_data_handlers[1], &gq7_filter_status);
    gq7_device->registerExtractor(gq7_filter_data_handlers[2], &gq7_filter_velocity_ned);
    gq7_device->registerExtractor(gq7_filter_data_handlers[3], &gq7_filter_velocity_ned_uncertainty);
    gq7_device->registerExtractor(gq7_filter_data_handlers[4], &gq7_filter_attitude_quaternion);
    gq7_device->registerExtractor(gq7_filter_data_handlers[5], &gq7_filter_dual_antenna_status);

    //Temporary for testing, record factory support
    if (mip::commands_3dm::factoryStreaming(*gq7_device, mip::commands_3dm::FactoryStreaming::Action::MERGE, 0) != CmdResult::ACK_OK)
        exit_gracefully("Couldn't set factory streaming");


    //
    //Resume the device
    //

    if(commands_base::resume(*gq7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the GQ7!");


    //
    //CV7 Setup
    //

    printf("Connecting to and configuring CV7.\n");

    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(commands_base::ping(*cv7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping CV7!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(commands_base::setIdle(*cv7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set CV7 to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    //TODO: Currently nacks due to firmware not being complete. Add this back when it no longer nacks
    //if(commands_3dm::defaultDeviceSettings(*cv7_device) != CmdResult::ACK_OK)
    //    exit_gracefully("ERROR: Could not load default CV7 settings!");


    //
    //Setup FILTER data format
    //

    uint16_t cv7_filter_base_rate;

    if(commands_3dm::getBaseRate(*cv7_device, data_filter::DESCRIPTOR_SET, &cv7_filter_base_rate) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not get filter base rate format from CV7!");

    const uint16_t cv7_filter_sample_rate = 100; // Hz
    const uint16_t cv7_filter_decimation = cv7_filter_base_rate / cv7_filter_sample_rate;

    std::array<DescriptorRate, 5> cv7_filter_descriptors = {{
        { data_shared::DATA_GPS_TIME,          cv7_filter_decimation },
        { data_filter::DATA_FILTER_STATUS,     cv7_filter_decimation },
        { data_filter::DATA_POS_LLH,           cv7_filter_decimation },
        { data_filter::DATA_VEL_NED,           cv7_filter_decimation },
        { data_filter::DATA_ATT_EULER_ANGLES,  cv7_filter_decimation },
    }};

    if(commands_3dm::writeMessageFormat(*cv7_device, data_filter::DESCRIPTOR_SET, cv7_filter_descriptors.size(), cv7_filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format on CV7!");


    //
    //Configure the PPS source to GPIO, the CV7 should be connected to the CV7 on GPIO pin 1
    //

    if(commands_3dm::writePpsSource(*cv7_device, mip::commands_3dm::PpsSource::Source::GPIO) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set PPS source on CV7");
    if(commands_3dm::writeGpioConfig(*cv7_device, 1, mip::commands_3dm::GpioConfig::Feature::PPS,
       mip::commands_3dm::GpioConfig::Behavior::PPS_INPUT, mip::commands_3dm::GpioConfig::PinMode::NONE) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set Pin 1 on CV7 to GPIO");

    //
    //TODO: Declination source
    //      Still need to figure out what to set it to
    //

    //
    //Filter Initialization Configuration
    //

    float cv7_filter_init_pos[3] = {0};
    float cv7_filter_init_vel[3] = {0};

    commands_filter::InitializationConfiguration::AlignmentSelector cv7_alignment;
    cv7_alignment = cv7_alignment.EXTERNAL;

    if(commands_filter::writeInitializationConfiguration(*cv7_device, 0,
       mip::commands_filter::InitializationConfiguration::InitialConditionSource::AUTO_POS_VEL_ATT,
       cv7_alignment, 0, 0, 0, cv7_filter_init_pos, cv7_filter_init_vel, commands_filter::FilterReferenceFrame::LLH) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure EKF on CV7!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(commands_filter::reset(*cv7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter on CV7!");


    //
    //Frame config
    //

    //Sensor ID 1 will be for measurements measured from the GQ7 itself
    uint8_t gq7_filter_sensor_id = 1;
    float cv7_to_gq7_rotation_quat[4] = { cv7_to_gq7_rotation[0], cv7_to_gq7_rotation[1], cv7_to_gq7_rotation[2], 0 };
    if(commands_aiding::writeReferenceFrame(*cv7_device, gq7_filter_sensor_id, mip::commands_aiding::ReferenceFrame::Format::EULER,
       cv7_to_gq7_translation, cv7_to_gq7_rotation_quat) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure frame id on CV7 for filter measurements from GQ7");
    // TODO: Doesn't appear to be implemented yet
    //if(commands_aiding::writeSensorFrameMapping(*cv7_device, gq7_filter_sensor_id, gq7_filter_sensor_id) != CmdResult::ACK_OK)
    //    exit_gracefully("ERROR: Unable to map frame id on CV7 for filter measurements from GQ7");

    //Sensor ID 2 will be for measurements measured from GNSS antenna 1 on the GQ7
    //TODO: Rotate the antenna offset by the GQ7 rotation
    uint8_t gq7_gnss_sensor_id = 2;
    float cv7_to_gnss_translation[3] =
    {
        cv7_to_gq7_translation[0] + gq7_gnss1_antenna_offset_meters[0],
        cv7_to_gq7_translation[1] + gq7_gnss1_antenna_offset_meters[1],
        cv7_to_gq7_translation[2] + gq7_gnss1_antenna_offset_meters[2],
    };
    if(commands_aiding::writeReferenceFrame(*cv7_device, gq7_gnss_sensor_id, mip::commands_aiding::ReferenceFrame::Format::EULER,
       cv7_to_gnss_translation, cv7_to_gq7_rotation_quat) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure frame id on CV7 for GNSS measurements from GQ7");
    // TODO: Doesn't appear to be implemented yet
    //if(commands_aiding::writeSensorFrameMapping(*cv7_device, gq7_gnss_sensor_id, gq7_gnss_sensor_id) != CmdResult::ACK_OK)
    //    exit_gracefully("ERROR: Unable to map frame id on CV7 for GNSS measurements from GQ7");


    //
    //Aiding measurement response mode
    //
    if (commands_aiding::writeAidingEchoControl(*cv7_device, mip::commands_aiding::AidingEchoControl::Mode::RESPONSE) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Unable to configure aiding echo control on CV7");


    //
    // Register data callbacks
    //

    //Filter Data
    DispatchHandler cv7_filter_data_handlers[5];

    cv7_device->registerExtractor(cv7_filter_data_handlers[0], &cv7_filter_gps_time, data_filter::DESCRIPTOR_SET);
    cv7_device->registerExtractor(cv7_filter_data_handlers[1], &cv7_filter_status);
    cv7_device->registerExtractor(cv7_filter_data_handlers[2], &cv7_filter_position_llh);
    cv7_device->registerExtractor(cv7_filter_data_handlers[3], &cv7_filter_velocity_ned);
    cv7_device->registerExtractor(cv7_filter_data_handlers[4], &cv7_filter_euler_angles);

    //Temporary for testing, record factory support
    if (mip::commands_3dm::factoryStreaming(*cv7_device, mip::commands_3dm::FactoryStreaming::Action::MERGE, 0) != CmdResult::ACK_OK)
        exit_gracefully("Couldn't set factory streaming");

    //
    //Resume the device
    //

    if(commands_base::resume(*cv7_device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the GQ7!");


    //
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();

    double cv7_prev_time_update_time = 0;

    float gq7_filter_dual_antenna_status_last_tow = 0;

    printf("Sensor is configured... waiting for filter to enter Full Navigation mode.\n");

    while(running)
    {
        gq7_device->update();
        cv7_device->update();

        //Check GQ7 GNSS fix and alert the user when it becomes valid
        if(!gq7_gnss_fix_info_valid)
        {
            gq7_gnss_fix_info_valid = gq7_gnss_fix_info.fix_type == data_gnss::FixInfo::FixType::FIX_3D ||
                                  gq7_gnss_fix_info.fix_type == data_gnss::FixInfo::FixType::FIX_RTK_FLOAT ||
                                  gq7_gnss_fix_info.fix_type == data_gnss::FixInfo::FixType::FIX_RTK_FIXED;
            if(gq7_gnss_fix_info_valid)
                printf("NOTE: GQ7 GNSS1 fix info valid\n");
        }

        //Check GQ7 Filter State
        if(!gq7_filter_state_full_nav)
        {
            gq7_filter_state_full_nav = gq7_filter_status.filter_state == data_filter::FilterMode::FULL_NAV;
            if (gq7_filter_state_full_nav)
                printf("NOTE: GQ7 Filter has entered full navigation mode.\n");
        }

        //Check GQ7 dual antenna status
        if(!gq7_filter_dual_antenna_status_fix)
        {
            gq7_filter_dual_antenna_status_fix = gq7_filter_dual_antenna_status.valid_flags == 1 &&
                                             (
                                                gq7_filter_dual_antenna_status.fix_type == data_filter::GnssDualAntennaStatus::FixType::FIX_DA_FIXED ||
                                                gq7_filter_dual_antenna_status.fix_type == data_filter::GnssDualAntennaStatus::FixType::FIX_DA_FLOAT
                                             );
            if(gq7_filter_dual_antenna_status_fix)
                printf("NOTE: Dual antenna fix acheived\n");
        }

        //Check CV7 Filter status
        if(!cv7_filter_state_full_nav)
        {
            cv7_filter_state_full_nav = cv7_filter_status.filter_state == data_filter::FilterMode::FULL_NAV;
            if (cv7_filter_state_full_nav)
                printf("NOTE: CV7-INS has entered full navigation mode\n");
        }

        //Determine if we need to send a time update command
        //  We should send a time update command if it has been a second since we last sent the time update, and the time is within 0.25 seconds of the top of the second
        //  We will also only send a time update if we have a GPS fix on antenna 1
        const double gq7_gps_time_seconds = gps_time_to_seconds(gq7_gnss_gps_time.week_number, gq7_gnss_gps_time.tow);
        if(gq7_gnss_fix_info_valid && gq7_gps_time_seconds - cv7_prev_time_update_time > 0.9 && gq7_gps_time_seconds - round(gq7_gps_time_seconds) < 0.25)
        {
            const bool time_update_succeeded =
                mip::commands_base::writeGpsTimeUpdate(*cv7_device, mip::commands_base::GpsTimeUpdate::FieldId::WEEK_NUMBER, gq7_gnss_gps_time.week_number) == CmdResult::ACK_OK &&
                mip::commands_base::writeGpsTimeUpdate(*cv7_device, mip::commands_base::GpsTimeUpdate::FieldId::TIME_OF_WEEK, gq7_gnss_gps_time.tow) == CmdResult::ACK_OK;
            if(time_update_succeeded)
                cv7_prev_time_update_time = gq7_gps_time_seconds;
            else
                printf("WARNING: Failed to send GPS time update to CV7-INS\n");
        }

        //If the fix info is valid, send the GNSS information to the CV7-INS
        if(gq7_gnss_fix_info_valid)
        {
            //Make a time object for the GNSS aiding commands
            mip::commands_aiding::Time gnss_external_time;
            gnss_external_time.timebase = mip::commands_aiding::Time::Timebase::EXTERNAL_TIME;
            gnss_external_time.nanoseconds = gq7_gps_time_seconds * 1e09;

            //LLH
            float llh_uncertainty[3] = { gq7_gnss_pos_llh.horizontal_accuracy, gq7_gnss_pos_llh.horizontal_accuracy, gq7_gnss_pos_llh.vertical_accuracy };
            if(mip::commands_aiding::llhPos(*cv7_device, gnss_external_time, gq7_gnss_sensor_id, gq7_gnss_pos_llh.latitude, gq7_gnss_pos_llh.longitude, gq7_gnss_pos_llh.msl_height,
               llh_uncertainty, mip::commands_aiding::LlhPos::ValidFlags::ALL) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external LLH position to CV7-INS\n");

            //NED Velocity
            float vel_ned_uncertainty[3] = { gq7_gnss_vel_ned.speed_accuracy, gq7_gnss_vel_ned.speed_accuracy, gq7_gnss_vel_ned.speed_accuracy };
            if(mip::commands_aiding::nedVel(*cv7_device, gnss_external_time, gq7_gnss_sensor_id, gq7_gnss_vel_ned.v, vel_ned_uncertainty, mip::commands_aiding::NedVel::ValidFlags::ALL) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external NED Velocity to CV7-INS\n");
        }

        //Make a time object for the Filter aiding commands
        mip::commands_aiding::Time filter_external_time;
        filter_external_time.timebase = mip::commands_aiding::Time::Timebase::EXTERNAL_TIME;
        filter_external_time.nanoseconds = gps_time_to_seconds(gq7_filter_gps_time.week_number, gq7_filter_gps_time.tow) * 1e09;

        //If the GQ7 is in full navigation mode, send the body frame velocity
        if(gq7_filter_state_full_nav)
        {
            //Use the attitude quaternion to rotate the NED velocity and uncertainty to the body frame
            float vehicle_fixed_frame_velocity[3];
            float vehicle_fixed_frame_velocity_uncertainty[3];
            float ned_velocity[3] = { gq7_filter_velocity_ned.north, gq7_filter_velocity_ned.east, gq7_filter_velocity_ned.down };
            float ned_velocity_uncertainty[3] = { gq7_filter_velocity_ned_uncertainty.north, gq7_filter_velocity_ned_uncertainty.east, gq7_filter_velocity_ned_uncertainty.down };
            rotate_vector_by_quaternion(ned_velocity, gq7_filter_attitude_quaternion.q, vehicle_fixed_frame_velocity);
            rotate_vector_by_quaternion(ned_velocity_uncertainty, gq7_filter_attitude_quaternion.q, vehicle_fixed_frame_velocity_uncertainty);
            if(mip::commands_aiding::vehicleFixedFrameVelocity(*cv7_device, filter_external_time, gq7_filter_sensor_id,
               vehicle_fixed_frame_velocity, vehicle_fixed_frame_velocity_uncertainty, mip::commands_aiding::VehicleFixedFrameVelocity::ValidFlags::ALL) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external body velocity to CV7-INS\n");
        }

        //If the GQ7 has a dual antenna fix, and the message has changed, send the dual antenna status message
        if(gq7_filter_dual_antenna_status_fix && gq7_filter_dual_antenna_status.time_of_week != gq7_filter_dual_antenna_status_last_tow)
        {
            gq7_filter_dual_antenna_status_last_tow = gq7_filter_dual_antenna_status.time_of_week;
            if(mip::commands_aiding::trueHeading(*cv7_device, filter_external_time, gq7_filter_sensor_id,
               gq7_filter_dual_antenna_status.heading, gq7_filter_dual_antenna_status.heading_unc, 1) != CmdResult::ACK_OK)
                printf("WARNING: Failed to send external dual antenna heading to CV7-INS\n");
        }

        //Once in full nav, print out data at 1 Hz
        mip::Timestamp curr_timestamp = getCurrentTimestamp();
        if(curr_timestamp - prev_print_timestamp >= 1000)
        {
            const double cv7_filter_gps_time_seconds = gps_time_to_seconds(cv7_filter_gps_time.week_number, cv7_filter_gps_time.tow);
            if(cv7_filter_state_full_nav)
            {
                printf("GPS_TIME = %.06f, FILTER_STATUS(FLAGS) = %d (%d), POS_LLH = [%f, %f, %f], VEL_NED = [%f, %f, %f], EULER_ANGLES = [%f, %f, %f]\n",
                    cv7_filter_gps_time_seconds,
                    (int)cv7_filter_status.filter_state, (int)cv7_filter_status.status_flags,
                    cv7_filter_position_llh.latitude, cv7_filter_position_llh.longitude, cv7_filter_position_llh.ellipsoid_height,
                    cv7_filter_velocity_ned.north, cv7_filter_velocity_ned.east, cv7_filter_velocity_ned.down,
                    cv7_filter_euler_angles.roll, cv7_filter_euler_angles.pitch, cv7_filter_euler_angles.yaw);
            }
            else
            {
                printf("GPS_TIME = %.06f, FILTER_STATUS(FLAGS) = %d (%d)\n",
                    cv7_filter_gps_time_seconds,
                    (int)cv7_filter_status.filter_state, (int)cv7_filter_status.status_flags);
            }
            prev_print_timestamp = curr_timestamp;
        }

        running = !should_exit();
    }

    exit_gracefully("Example Completed Successfully.");
}


////////////////////////////////////////////////////////////////////////////////
// GPS time to UTC time Function
////////////////////////////////////////////////////////////////////////////////

double gps_time_to_seconds(const double week_number, const double time_of_week)
{
    return week_number * 604800.0 + time_of_week;
}

////////////////////////////////////////////////////////////////////////////////
// NED velocity to body velocity Function
////////////////////////////////////////////////////////////////////////////////

void quat_inverse(const float* q, float* result)
{
    result[0] = q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];
}

void quat_times_vec(const float* q, const float* v, float* result)
{
    result[0] = -q[1] * v[0] - q[2] * v[1] - q[3] * v[2];
    result[1] = q[0] * v[0] + q[2] * v[2] - q[3] * v[1];
    result[2] = q[0] * v[1] + q[3] * v[0] - q[1] * v[2];
    result[3] = q[0] * v[2] + q[1] * v[1] - q[2] * v[0];
}

void quat_times_quat(const float* q1, const float* q2, float* result)
{
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    result[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
    result[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
}

void rotate_vector_by_quaternion(const float* vector_in, const float* quaternion, float* vector_out)
{
    //Get the inverse of the attitude quaternion
    float quaternion_inverse[4];
    quat_inverse(quaternion, quaternion_inverse);

    //Multiply the quaternion by the velocity
    float quaternion_inverse_x_vector_in[4];
    quat_times_vec(quaternion_inverse, vector_in, quaternion_inverse_x_vector_in);

    //Multiply the quaternion by the inverse
    float vector_out_quat[4];
    quat_times_quat(quaternion_inverse_x_vector_in, quaternion, vector_out_quat);

    //Copy the x, y, and z values into the body velocity
    vector_out[0] = vector_out_quat[1];
    vector_out[1] = vector_out_quat[2];
    vector_out[2] = vector_out_quat[3];
}

////////////////////////////////////////////////////////////////////////////////
// Print Usage Function
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0)
{
    printf("Usage: %s <port> <baudrate>\n", argv0);
    return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Exit Function
////////////////////////////////////////////////////////////////////////////////

void exit_gracefully(const char *message)
{
    if(message)
        printf("%s\n", message);

#ifdef _WIN32
    int dummy = getchar();
#endif

    exit(0);
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
  return false;

}

