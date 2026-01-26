/*
 * Based on TransceiverUHD/runTransceiver.cpp
 *
 * OpenBTS provides an open source alternative to legacy telco protocols and
 * traditionally complex, proprietary hardware systems.
 *
 * Copyright 2008, 2009 Free Software Foundation, Inc.
 * Copyright 2010 Kestrel Signal Processing, Inc.
 * Copyright 2014 Range Networks, Inc.
 * Copyright 2026 Late Beam
 * 
 * This software is distributed under the terms of the GNU General Public 
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 * 
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <Logger.h>
#include <UMTSCommon.h>
#include <Configuration.h>

#include "Transceiver.h"
#include "PCIeSDRDevice.h"

/* Default maximum expected delay spread in symbols */
#define DEFAULT_MAX_DELAY         50

/* Sample rate for all devices */
#define DEVICE_RATE               6.25e6

ConfigurationKeyMap getConfigurationKeys2();

ConfigurationTable gConfig("./OpenBTS-UMTS.db",
                           "Open", getConfigurationKeys2());

volatile bool gbShutdown = false;

static void shutdown_handler(int signo)
{
  std::cout << std::endl << "** Received shutdown signal" << std::endl;
  gbShutdown = true;
}

static void register_signal_handlers()
{
  if (signal(SIGINT, shutdown_handler) == SIG_ERR) {
    std::cerr << "** Failed to install SIGINT signal handler" << std::endl;
    exit(1);
  }

  if (signal(SIGTERM, shutdown_handler) == SIG_ERR) {
    std::cerr << "** Failed to install SIGTERM signal handler" << std::endl;
    exit(1);
  }
}

/* Logging check in the configuration is mandatory */
static bool init_config()
{
  
  try {
    std::cout << "** Configuring logger" << std::endl;
    gLogInitTransceiver("transceiver", gConfig.getStr("Log.LevelTransceiver").c_str(), LOG_LOCAL7);
  } catch (ConfigurationTableKeyNotFound e) {
    LOG(EMERG) << "** Required configuration parameter " << e.key()
               << " not defined, aborting";
    return false;
  }

  return true;
}

/* Optional expected delay spread (default 50 symbols) */
static int init_max_delay()
{
  int max_delay;

  try {
    max_delay = gConfig.getNum("UMTS.Radio.MaxExpectedDelaySpread");
  } catch (ConfigurationTableKeyNotFound e) {
    max_delay = DEFAULT_MAX_DELAY;
  }

  return max_delay;
}

/* Optional external reference enable (default off) */
static bool init_extref()
{
  int enable;

  try {
    enable = gConfig.getNum("TRX.Reference");
  } catch (ConfigurationTableKeyNotFound e) {
    enable = 0;
  }

  return enable != 0;
}

/* Optional device hint (default none) */
static std::string init_devaddr()
{
  std::string addr;

  try {
    addr = gConfig.getNum("UMTS.Radio.UHD.DeviceAddress");
  } catch (ConfigurationTableKeyNotFound e) {
    addr = "";
  }

  return addr;
}

// tässä tulee index
int main(int argc, char *argv[])
{
  PCIeSDRDevice *sdr_device = NULL;
  RadioDevice *dev = NULL;
  Transceiver *trx = NULL;
  RadioInterface *radio = NULL;

  int max_delay;
  bool found, extref;
  std::string devaddr;

  int transceiver_index = 0; // Default: 0 is ok index

  std::cout << "argc = " << argc << std::endl;
  std::vector<std::string> args(argv + 1, argv + argc);
  
  // Quick hack for parsing as there are only 2 params and the first is not used
  for (size_t i = 0; i < args.size(); ++i)
  {
    if (i == 1)
      transceiver_index = std::stoi(args[i]);
  }
  std::cout << "transceiver_index = " << transceiver_index << std::endl;

  /* Capture termination signals */
  register_signal_handlers();

  /* Fail if we don't have logging */
  if (!init_config())
    goto shutdown;

  /* Optional parameters */
  max_delay = init_max_delay();
  extref = init_extref();
  devaddr = init_devaddr();

  srandom(time(NULL));

  if (extref)
    std::cout << "** Using external clock reference" << std::endl;
  else
    std::cout << "** Using internal clock reference" << std::endl;

  std::cout << "** Searching for USRP device " << devaddr << std::endl;
  sdr_device = new PCIeSDRDevice(DEVICE_RATE);
  found = sdr_device->open(devaddr, extref,transceiver_index);

  if (found) {
    std::cout << "** Device ready" << std::endl;
    dev = (RadioDevice *) sdr_device;
  } else {
    std::cout << "** Device not available" << std::endl;
    goto shutdown;
  }

  radio = new RadioInterface(dev, 0);
  if (!radio->init()) {
    std::cout << "** Radio failed to initialize" << std::endl;
    goto shutdown;
  }

  trx = new Transceiver(gConfig.getNum("TRX.Port"), "127.0.0.1", UMTS::Time(4, 0), radio);
  trx->receiveFIFO(radio->receiveFIFO());
  trx->init(max_delay);

  while (!gbShutdown)
    sleep(1);

shutdown:
  delete trx;
  delete radio;
  delete sdr_device;

  return 0;
}

ConfigurationKeyMap getConfigurationKeys2()
{
	extern ConfigurationKeyMap getConfigurationKeys();
	ConfigurationKeyMap map = getConfigurationKeys();
	ConfigurationKey *tmp;

	tmp = new ConfigurationKey("TRX.RadioFrequencyOffset","128",
		"~170Hz steps",
		ConfigurationKey::FACTORY,
		ConfigurationKey::VALRANGE,
		"96:160",// educated guess
		true,
		"Fine-tuning adjustment for the transceiver master clock.  "
			"Roughly 170 Hz/step.  "
			"Set at the factory.  "
			"Do not adjust without proper calibration."
	);
	map[tmp->getName()] = *tmp;
	delete tmp;

	tmp = new ConfigurationKey("TRX.TxAttenOffset","0",
		"dB of attenuation",
		ConfigurationKey::FACTORY,
		ConfigurationKey::VALRANGE,
		"0:100",// educated guess
		true,
		"Hardware-specific gain adjustment for transmitter, matched to the power amplifier, expessed as an attenuationi in dB.  "
			"Set at the factory.  "
			"Do not adjust without proper calibration."
	);
	map[tmp->getName()] = *tmp;
	delete tmp;

	tmp = new ConfigurationKey("TRX.RadioNumber","0",
		"",
		ConfigurationKey::FACTORY,
		ConfigurationKey::VALRANGE,
		"0:9",		// Not likely to have 10 radios on the same computer.  Not likely to have >1
		true,
		"If non-0, use multiple radios on the same cpu, numbered 1-9.  Must change TRX.Port also.  Provide a separate config file for each OpenBTS+Radio combination using the environment variable or --config command line option."
	);
	map[tmp->getName()] = *tmp;
	delete(tmp);

	return map;
}
