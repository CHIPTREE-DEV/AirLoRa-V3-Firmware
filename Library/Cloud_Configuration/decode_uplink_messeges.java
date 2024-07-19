function decodeUplink(input) {
  return {
    data: {
      bytes: input.bytes,
      battery: input.bytes[0],
      temperature: input.bytes[1],
      PM10: input.bytes[2],
      PM2_5: input.bytes[3],
      PM1: input.bytes[4],
      CO2: input.bytes[5],
      CO: input.bytes[6],
      NO2: input.bytes[7],
      O3: input.bytes[8],
      SO2: input.bytes[9],
    },
    warnings: [],
    errors: []
  };
}