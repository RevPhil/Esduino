void loop()
{
  // if any rxBytes are available, process them
  processRxData(rxBytes);

  //***********************************************************//
  // YOU CAN ADD YOUR OWN CODE HERE SUCH AS CODE TO READ       //
  // A TEMPERATURE AND/OR OTHER SENSOR(S). JUST MAKE SURE      //
  // YOUR FUNCTIONS DON'T BLOCK UNECESSARILLY OR YOU WILL      //
  // SLOW DOWN THE SYSTEM'S RESPONSE TO INCOMMING I2C COMMANDS //
  // OR CAUSE TIMEOUTS AT THE HOST END!!!!!                    //
  //***********************************************************//

}//END OF loop()
