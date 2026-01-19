const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const utils = require('zigbee-herdsman-converters/lib/utils');

const e = exposes.presets;
const ea = exposes.access;

const readMeteringScale = async (endpoint) => {
  await endpoint.read('seMetering', [
    'multiplier',
    'divisor',
    'summationFormatting',
    'demandFormatting',
    'unitOfMeasure',
  ]);
};

const readMeteringValues = async (endpoint) => {
  await endpoint.read('seMetering', ['currentSummationDelivered', 'instantaneousDemand']);
};

const MFG_CLUSTER = 0xFD10;
const ATTR = {
  reset_counter: 0x0008,
};

const ATTR_TYPE = {
  reset_counter: 0x10, // boolean
};

const hasBatteryPower = (device) => {
  if (typeof device?.powerSource === 'number') {
    return device.powerSource === 0x03; // Battery
  }

  const src = (device?.powerSource || '').toString().toLowerCase();
  if (!src) return false;
  if (src.includes('battery')) return true;
  if (src.includes('dc')) return false;
  if (src.includes('mains')) return false;
  if (src.includes('ac')) return false;
  return false;
};

const tzLocal = {
  reset_action: {
    key: ['reset_counter'],
    convertSet: async (entity, key, value, meta) => {
      if (value === null || value === undefined || value === '') {
        return {state: {reset_counter: null}};
      }

      const endpoint = meta.device?.getEndpoint ? (meta.device.getEndpoint(1) || entity) : entity;

      const payload = {
        [ATTR[key]]: {value: 1, type: ATTR_TYPE[key]},
      };

      await endpoint.write(MFG_CLUSTER, payload, {
        manufacturerCode: 0x1234,
        disableDefaultResponse: true,
      });

      return {state: {reset_counter: null}};
    },
  },
};

const applyHaMeta = (expose, deviceClass, stateClass) => {
  if (deviceClass !== undefined) {
    if (typeof expose.withDeviceClass === 'function') {
      expose.withDeviceClass(deviceClass);
    } else {
      expose.device_class = deviceClass;
    }
  }

  if (stateClass !== undefined) {
    if (typeof expose.withStateClass === 'function') {
      expose.withStateClass(stateClass);
    } else {
      expose.state_class = stateClass;
    }
  }

  return expose;
};

const DEVICE_VARIANTS = [
  {id: 'ESP32-PulseMeter-Gas', energyUnit: 'm³', powerUnit: 'm³/h', desc: 'Zigbee gas pulse meter', type: 'gas'},
  {id: 'ESP32-PulseMeter-Water', energyUnit: 'm³', powerUnit: 'm³/h', desc: 'Zigbee water pulse meter', type: 'water'},
  {id: 'ESP32-PulseMeter-Electric', energyUnit: 'kWh', powerUnit: 'kW', desc: 'Zigbee electric pulse meter', type: 'electric'},
];

const logWarn = (meta, message, details) => {
  if (meta?.logger?.warn) {
    meta.logger.warn(message, details);
  } else {
    console.warn(message, details);
  }
};

const safeRound = (value, precision, meta) => {
  if (value === undefined || value === null) return value;

  const numeric = typeof value === 'bigint' ? Number(value) : value;

  if (typeof numeric !== 'number' || Number.isNaN(numeric) || !Number.isFinite(numeric)) {
    logWarn(meta, 'Ignoring invalid metering value', {value});
    return undefined;
  }

  try {
    return utils.precisionRound(numeric, precision);
  } catch (error) {
    logWarn(meta, 'precisionRound failed', {value: numeric, error: error?.message || error});
    return undefined;
  }
};

const buildExposes = (variant, batteryCapable = true) => {
  const reset = exposes.enum('reset_counter', ea.SET, ['RESET'])
      .withDescription('Reset counter (write-only)');

  if (variant.type === 'electric') {
    const energy = e.energy().withAccess(ea.STATE).withValueStep(0.001);
    const power = e.power().withAccess(ea.STATE).withValueStep(0.001);
    const exposeList = [energy, power];

    if (typeof energy.withUnit === 'function') {
      energy.withUnit(variant.energyUnit);
    }
    if (typeof power.withUnit === 'function') {
      power.withUnit(variant.powerUnit);
    }

    if (batteryCapable) {
      const battVoltage = exposes.numeric('battery_voltage', ea.STATE)
          .withProperty('voltage')
          .withDescription('Battery voltage')
          .withUnit('V')
          .withValueMin(0)
          .withValueStep(0.1);
      applyHaMeta(battVoltage, 'voltage', 'measurement');
      exposeList.push(e.battery(), battVoltage);
    }

    exposeList.push(reset);
    return exposeList;
  }

  const isGas = variant.type === 'gas';

  const total = exposes.numeric(isGas ? 'gas' : 'water', ea.STATE)
      .withProperty('energy')
      .withDescription('Total consumption')
      .withUnit(variant.energyUnit)
      .withValueMin(0)
      .withValueStep(0.001);

  applyHaMeta(total, isGas ? 'gas' : 'water', 'total_increasing');

  const flow = exposes.numeric(isGas ? 'gas_flow' : 'water_flow', ea.STATE)
      .withProperty('power')
      .withDescription('Flow rate')
      .withUnit(variant.powerUnit)
      .withValueMin(0)
      .withValueStep(0.001);

  applyHaMeta(flow, 'volume_flow_rate', 'measurement');

  const exposeList = [total, flow];

  if (batteryCapable) {
    const battVoltage = exposes.numeric('battery_voltage', ea.STATE)
        .withProperty('voltage')
        .withDescription('Battery voltage')
        .withUnit('V')
        .withValueMin(0)
        .withValueStep(0.1);
    applyHaMeta(battVoltage, 'voltage', 'measurement');
    exposeList.push(e.battery(), battVoltage);
  }

  exposeList.push(reset);
  return exposeList;
};

const FLOW_MODELS = new Set(['ESP32-PulseMeter-Gas', 'ESP32-PulseMeter-Water']);

const fzLocal = {
  metering_round: {
    ...fz.metering,
    convert: (model, msg, publish, options, meta) => {
      let res;
      try {
        res = fz.metering.convert(model, msg, publish, options, meta);
      } catch (error) {
        logWarn(meta, 'metering.convert failed', {error: error?.message || error});
        return;
      }

      if (!res) return res;

      const flowVariant =
        FLOW_MODELS.has(model?.model) ||
        FLOW_MODELS.has(meta?.device?.modelId) ||
        FLOW_MODELS.has(msg?.device?.modelId);

      if (res.energy !== undefined && res.energy !== null) {
        const energy = safeRound(res.energy, 3, meta);
        if (energy !== undefined) res.energy = energy;
        else delete res.energy;
      }

      if (res.power !== undefined && res.power !== null) {
        const rawPower = flowVariant ? res.power / 1000 : res.power;
        const power = safeRound(rawPower, 3, meta);
        if (power !== undefined) res.power = power;
        else delete res.power;
      }

      return res;
    },
  },
};

module.exports = DEVICE_VARIANTS.map((variant) => ({
  zigbeeModel: [variant.id],
  model: variant.id,
  vendor: 'Custom',
  description: variant.desc,

  fromZigbee: [fzLocal.metering_round, fz.battery],
  toZigbee: [tzLocal.reset_action],

  meta: {
    configureKey: 15,
    manufacturerCode: 0x1234,
  },

  configure: async (device, coordinatorEndpoint, logger) => {
    const endpoint = device.getEndpoint(1);
    const batteryCapable = hasBatteryPower(device);
    const bindClusters = ['seMetering'];
    if (batteryCapable) bindClusters.unshift('genPowerCfg');

    await reporting.bind(endpoint, coordinatorEndpoint, bindClusters);

    await reporting.instantaneousDemand(endpoint, {min: 10, max: 300, change: 0});
    await reporting.currentSummDelivered(endpoint, {min: 10, max: 300, change: 0});
    if (batteryCapable) {
      await reporting.batteryPercentageRemaining(endpoint, {min: 10, max: 21600, change: 0});
      await reporting.batteryVoltage(endpoint, {min: 10, max: 21600, change: 0});
    } else if (logger?.info) {
      logger.info(`Skipping battery reporting (powerSource=${device?.powerSource || 'unknown'})`);
    }

    await readMeteringScale(endpoint);
    await readMeteringValues(endpoint);
  },

  exposes: (device) => buildExposes(variant, hasBatteryPower(device)),
  ota: true,
}));
