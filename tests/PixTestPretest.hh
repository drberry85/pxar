#ifndef PIXTESTPRETEST_H
#define PIXTESTPRETEST_H

#include "PixTest.hh"

class DLLEXPORT PixTestPretest: public PixTest {
public:
  PixTestPretest(PixSetup *, std::string);
  PixTestPretest();
  virtual ~PixTestPretest();
  virtual bool setParameter(std::string parName, std::string sval);
  void init();
  void setToolTips();
  void runCommand(std::string);
  void bookHist(std::string);

  void doTest();
  void setVana();
  void programROC();
  /// Doug's timing setting
  void setTimings();
  /// Wolfram's timing setting (optimized)
  void findTiming();
  void findWorkingPixel();  
  void setVthrCompCalDel();
  void setVthrCompId();
  void setCalDel();

  void setApi(pxar::pxarCore * api, PixSetup * setup );
  int test_timing(int nloop, int d160, int d400, int rocdelay=-1, int htdelay=0, int tokdelay=0);
  int find_timing(int npass=0);
  bool find_midpoint(int threshold, int data[], uint8_t & position, int & width);
  bool find_midpoint(int threshold, double step, double range,  int data[], uint8_t & position, int & width);
  int tbmget(std::string name, const uint8_t core, uint8_t & value);
  int tbmsetbit(std::string name, uint8_t coreMask, int bit, int value);
  int tbmset(int address, int value);
  int tbmset(std::string name, uint8_t coreMask, int value, uint8_t valueMask=0xff);
  int countGood(unsigned int nloop, unsigned int ntrig, int ftrigkhz, int nroc);
  int countErrors(unsigned int ntrig=1, int ftrigkhz=0, int nroc_expected=-1, bool setup=true);
  int getBuffer(std::vector<uint16_t> & buf);
  int getData(std::vector<uint16_t> & buf, std::vector<DRecord > & data, int verbosity=1, int nroc_expected=-1, bool resetStats=true);
  int setupDaq(int ntrig, int ftrigkhz, int verbosity=0);
  int runDaq(std::vector<uint16_t> & buf, int ntrig, int ftrigkhz, int verbosity=0, bool setup=true);
  int runDaq(int ntrig, int ftrigkhz, int verbosity=0);
  int resetDaqStatus();
  int pg_sequence(int seq, int length=0);
  int pg_restore();
  std::vector<unsigned int> fDaqChannelRocIdOffset;
  int rocIdFromReadoutPosition(unsigned int daqChannel, unsigned int roc){
    return fDaqChannelRocIdOffset[daqChannel]+roc;
  }
  bool layer1(){return false;};
  std::vector<std::pair<std::string,uint8_t> > fSigdelays;
  std::vector<std::pair<std::string,uint8_t> > fSigdelaysSetup;

  bool verbose;
  bool fIgnoreReadbackErrors;
  bool tbm08(){ return fApi->_dut->getTbmType()=="tbm08c"; };
  bool tbmWithDummyHits(){ return !tbm08(); }
  std::stringstream out;
  int fDeser400XOR1sum[8];  // count transitions at the 8 phases
  int fDeser400XOR2sum[8];
  int fDeser400err;
  int fPrerun;
  int fGetBufMethod;
  unsigned int fPeriod;
  unsigned int fTCT, fTRC, fTTK;
  unsigned int fNumberOfEvents;
  unsigned int fHeaderCount;
  unsigned int fnRocPerChannel;
  unsigned int fnDaqChannel;
  bool fFW35;  // for fw<=3.5, to be removed
  static const size_t nDaqChannelMax=8;
  unsigned int fDeser400XOR[nDaqChannelMax];
  unsigned int fDeser400SymbolErrors[nDaqChannelMax];
  unsigned int fDeser400PhaseErrors[nDaqChannelMax];
  unsigned int fDeser400XORChanges[nDaqChannelMax];
  unsigned int fRocReadBackErrors[nDaqChannelMax];
  unsigned int fNTBMHeader[nDaqChannelMax];
  unsigned int fDeser400_frame_error[nDaqChannelMax];
  unsigned int fDeser400_code_error[nDaqChannelMax];
  unsigned int fDeser400_idle_error[nDaqChannelMax];
  unsigned int fDeser400_trailer_error[nDaqChannelMax];
  int restoreDaq(int verbosity=0);
  unsigned int fSeq;
  std::vector<uint16_t> fBuf;
  std::vector<unsigned int> fHeadersWithErrors;
  unsigned int fBufsize;
  bool fPixelConfigNeeded;
  uint16_t fRocHeaderData[17];

private:

  int     fTargetIa;
  int     fNoiseWidth;
  int     fNoiseMargin;
  int     fIterations;
  int     fParNtrig;
  int     fParVcal, fParDeltaVthrComp;
  double  fParFracCalDel;
  int     fIgnoreProblems;

  ClassDef(PixTestPretest, 1)

};
#endif
