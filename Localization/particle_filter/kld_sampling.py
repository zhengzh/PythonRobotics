#/*********************************************************************
#
#  KLD-SAMPLING: Adequately Sampling from an Unknown Distribution.
#  
#  Copyright (C) 2006 - Patrick Beeson  (pbeeson@cs.utexas.edu)
#
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
#  USA
#
#*********************************************************************/

# Ported to python K Nickels 10/21/13

import random
import numpy as np
import math


class KLDSampling:
    ABSOLUTE_MIN = 10;
    """ A helper class for KLDSampling in Python """
#/**
#   Initialize a round of KLD sampling.  Takes in kld-parameters:
#   quantile, kld-error, bin size, minimum number of samples
#**/
    def __init__(self,quantile, err, bsz, sample_min) :

        # Constructs Z-table (from ztable.data) to lookup statistics.

        self.ztable = [ 0.0000000e+00,3.9900000e-03,7.9800000e-03,1.1970000e-02,1.5950000e-02,1.9940000e-02,2.3920000e-02,2.7900000e-02,3.1880000e-02,3.5860000e-02,3.9830000e-02,4.3800000e-02,4.7760000e-02,5.1720000e-02,5.5670000e-02,5.9620000e-02,6.3560000e-02,6.7490000e-02,7.1420000e-02,7.5350000e-02,7.9260000e-02,8.3170000e-02,8.7060000e-02,9.0950000e-02,9.4830000e-02,9.8710000e-02,1.0257000e-01,1.0642000e-01,1.1026000e-01,1.1409000e-01,1.1791000e-01,1.2172000e-01,1.2552000e-01,1.2930000e-01,1.3307000e-01,1.3683000e-01,1.4058000e-01,1.4431000e-01,1.4803000e-01,1.5173000e-01,1.5542000e-01,1.5910000e-01,1.6276000e-01,1.6640000e-01,1.7003000e-01,1.7364000e-01,1.7724000e-01,1.8082000e-01,1.8439000e-01,1.8793000e-01,1.9146000e-01,1.9497000e-01,1.9847000e-01,2.0194000e-01,2.0540000e-01,2.0884000e-01,2.1226000e-01,2.1566000e-01,2.1904000e-01,2.2240000e-01,2.2575000e-01,2.2907000e-01,2.3237000e-01,2.3565000e-01,2.3891000e-01,2.4215000e-01,2.4537000e-01,2.4857000e-01,2.5175000e-01,2.5490000e-01,2.5804000e-01,2.6115000e-01,2.6424000e-01,2.6730000e-01,2.7035000e-01,2.7337000e-01,2.7637000e-01,2.7935000e-01,2.8230000e-01,2.8524000e-01,2.8814000e-01,2.9103000e-01,2.9389000e-01,2.9673000e-01,2.9955000e-01,3.0234000e-01,3.0511000e-01,3.0785000e-01,3.1057000e-01,3.1327000e-01,3.1594000e-01,3.1859000e-01,3.2121000e-01,3.2381000e-01,3.2639000e-01,3.2894000e-01,3.3147000e-01,3.3398000e-01,3.3646000e-01,3.3891000e-01,3.4134000e-01,3.4375000e-01,3.4614000e-01,3.4849000e-01,3.5083000e-01,3.5314000e-01,3.5543000e-01,3.5769000e-01,3.5993000e-01,3.6214000e-01,3.6433000e-01,3.6650000e-01,3.6864000e-01,3.7076000e-01,3.7286000e-01,3.7493000e-01,3.7698000e-01,3.7900000e-01,3.8100000e-01,3.8298000e-01,3.8493000e-01,3.8686000e-01,3.8877000e-01,3.9065000e-01,3.9251000e-01,3.9435000e-01,3.9617000e-01,3.9796000e-01,3.9973000e-01,4.0147000e-01,4.0320000e-01,4.0490000e-01,4.0658000e-01,4.0824000e-01,4.0988000e-01,4.1149000e-01,4.1308000e-01,4.1466000e-01,4.1621000e-01,4.1774000e-01,4.1924000e-01,4.2073000e-01,4.2220000e-01,4.2364000e-01,4.2507000e-01,4.2647000e-01,4.2785000e-01,4.2922000e-01,4.3056000e-01,4.3189000e-01,4.3319000e-01,4.3448000e-01,4.3574000e-01,4.3699000e-01,4.3822000e-01,4.3943000e-01,4.4062000e-01,4.4179000e-01,4.4295000e-01,4.4408000e-01,4.4520000e-01,4.4630000e-01,4.4738000e-01,4.4845000e-01,4.4950000e-01,4.5053000e-01,4.5154000e-01,4.5254000e-01,4.5352000e-01,4.5449000e-01,4.5543000e-01,4.5637000e-01,4.5728000e-01,4.5818000e-01,4.5907000e-01,4.5994000e-01,4.6080000e-01,4.6164000e-01,4.6246000e-01,4.6327000e-01,4.6407000e-01,4.6485000e-01,4.6562000e-01,4.6638000e-01,4.6712000e-01,4.6784000e-01,4.6856000e-01,4.6926000e-01,4.6995000e-01,4.7062000e-01,4.7128000e-01,4.7193000e-01,4.7257000e-01,4.7320000e-01,4.7381000e-01,4.7441000e-01,4.7500000e-01,4.7558000e-01,4.7615000e-01,4.7670000e-01,4.7725000e-01,4.7778000e-01,4.7831000e-01,4.7882000e-01,4.7932000e-01,4.7982000e-01,4.8030000e-01,4.8077000e-01,4.8124000e-01,4.8169000e-01,4.8214000e-01,4.8257000e-01,4.8300000e-01,4.8341000e-01,4.8382000e-01,4.8422000e-01,4.8461000e-01,4.8500000e-01,4.8537000e-01,4.8574000e-01,4.8610000e-01,4.8645000e-01,4.8679000e-01,4.8713000e-01,4.8745000e-01,4.8778000e-01,4.8809000e-01,4.8840000e-01,4.8870000e-01,4.8899000e-01,4.8928000e-01,4.8956000e-01,4.8983000e-01,4.9010000e-01,4.9036000e-01,4.9061000e-01,4.9086000e-01,4.9111000e-01,4.9134000e-01,4.9158000e-01,4.9180000e-01,4.9202000e-01,4.9224000e-01,4.9245000e-01,4.9266000e-01,4.9286000e-01,4.9305000e-01,4.9324000e-01,4.9343000e-01,4.9361000e-01,4.9379000e-01,4.9396000e-01,4.9413000e-01,4.9430000e-01,4.9446000e-01,4.9461000e-01,4.9477000e-01,4.9492000e-01,4.9506000e-01,4.9520000e-01,4.9534000e-01,4.9547000e-01,4.9560000e-01,4.9573000e-01,4.9585000e-01,4.9598000e-01,4.9609000e-01,4.9621000e-01,4.9632000e-01,4.9643000e-01,4.9653000e-01,4.9664000e-01,4.9674000e-01,4.9683000e-01,4.9693000e-01,4.9702000e-01,4.9711000e-01,4.9720000e-01,4.9728000e-01,4.9736000e-01,4.9744000e-01,4.9752000e-01,4.9760000e-01,4.9767000e-01,4.9774000e-01,4.9781000e-01,4.9788000e-01,4.9795000e-01,4.9801000e-01,4.9807000e-01,4.9813000e-01,4.9819000e-01,4.9825000e-01,4.9831000e-01,4.9836000e-01,4.9841000e-01,4.9846000e-01,4.9851000e-01,4.9856000e-01,4.9861000e-01,4.9865000e-01,4.9869000e-01,4.9874000e-01,4.9878000e-01,4.9882000e-01,4.9886000e-01,4.9889000e-01,4.9893000e-01,4.9896000e-01,4.9900000e-01,4.9903000e-01,4.9906000e-01,4.9910000e-01,4.9913000e-01,4.9916000e-01,4.9918000e-01,4.9921000e-01,4.9924000e-01,4.9926000e-01,4.9929000e-01,4.9931000e-01,4.9934000e-01,4.9936000e-01,4.9938000e-01,4.9940000e-01,4.9942000e-01,4.9944000e-01,4.9946000e-01,4.9948000e-01,4.9950000e-01,4.9952000e-01,4.9953000e-01,4.9955000e-01,4.9957000e-01,4.9958000e-01,4.9960000e-01,4.9961000e-01,4.9962000e-01,4.9964000e-01,4.9965000e-01,4.9966000e-01,4.9968000e-01,4.9969000e-01,4.9970000e-01,4.9971000e-01,4.9972000e-01,4.9973000e-01,4.9974000e-01,4.9975000e-01,4.9976000e-01,4.9977000e-01,4.9978000e-01,4.9978000e-01,4.9979000e-01,4.9980000e-01,4.9981000e-01,4.9981000e-01,4.9982000e-01,4.9983000e-01,4.9983000e-01,4.9984000e-01,4.9985000e-01,4.9985000e-01,4.9986000e-01,4.9986000e-01,4.9987000e-01,4.9987000e-01,4.9988000e-01,4.9988000e-01,4.9989000e-01,4.9989000e-01,4.9990000e-01,4.9990000e-01,4.9990000e-01,4.9991000e-01,4.9991000e-01,4.9992000e-01,4.9992000e-01,4.9992000e-01,4.9992000e-01,4.9993000e-01,4.9993000e-01,4.9993000e-01,4.9994000e-01,4.9994000e-01,4.9994000e-01,4.9994000e-01,4.9995000e-01,4.9995000e-01,4.9995000e-01,4.9995000e-01,4.9995000e-01,4.9996000e-01,4.9996000e-01,4.9996000e-01,4.9996000e-01,4.9996000e-01,4.9996000e-01,4.9997000e-01,4.9997000e-01,4.9997000e-01,4.9997000e-01,4.9997000e-01,4.9997000e-01,4.9997000e-01,4.9997000e-01,4.9998000e-01,4.9998000e-01,4.9998000e-01,4.9998000e-01];

        self.support_samples=0;
        self.num_samples=0;

        if (sample_min < self.ABSOLUTE_MIN):
            self.kld_samples=self.ABSOLUTE_MIN;
        else:
            self.kld_samples=sample_min;

        self.bins = [[]]; # bins is a list of lists
                          # each list is an N-dim bin of support

        confidence=quantile-0.5; # ztable is from right side of mean
        confidence=min(0.49998,max(0,confidence));

        self.max_error=err;
        self.bin_size=bsz; # list of lists

        self.zvalue=4.1;
        for i in range(len(self.ztable)):
          if (self.ztable[i] >= confidence):
            self.zvalue=i/100.0;
            break;

        print "zvalue is ",self.zvalue;


#/**
#   Update kld-sampler with the last sample drawn.  Returns a guess at
#   the number of samples needed before the distribution (which is
#   unknown) is adequately sampled.
#
#   sample = list of lists - each item is a list of dims
#**/
    def update(self,sample) :
        #print "updating with samples"
        assert len(sample[0])==len(self.bin_size), \
               "kld-sampling: Sample size not the same number of dimensions as the bins\n";

        self.num_samples = self.num_samples+1;
        if self.in_empty_bin(sample):
            self.add_to_bins(sample);
            self.support_samples = self.support_samples+1;
            if self.support_samples>=2:
                k = self.support_samples-1;
                k=math.ceil(k/(2*self.max_error)*pow(1-2/(9.0*k)+math.sqrt(2/(9.0*k))*self.zvalue,3));
                if k>self.kld_samples:
                    self.kld_samples = k;

        #print "done updating";
        return self.kld_samples;

#/**
#   Determines whether a sample falls into a bin that has already been
#   sampled.
# **/
    def in_empty_bin(self,sample):
        # init case - bins are empty
        if len(self.bins)==0:
            return True;

        # check against existing bins
        curr_bin = [ math.floor(sample[0][i]/self.bin_size[i]) for i in range(len(self.bin_size))]
        if curr_bin in self.bins:
            #print "sample", sample," is in bin ",curr_bin," already";
            return False;
        else:
            #print "sample", sample," NOT in bins ",self.bins," yet";
            return True;

        print "done."

    def add_to_bins(self,sample):
        curr_bin = [ math.floor(sample[0][i]/self.bin_size[i]) for i in range(len(self.bin_size))]
        #print "adding sample", sample," to bins ",self.bins;
        #curr_bin = math.floor(sample/self.bin_size)
        self.bins.append(curr_bin);
        #print "done adding sample", sample," to bins ",self.bins;

# Return a point drawn from a ND Gaussian distribution centered at
# mean with a given standard deviation.
def get_sample(mean, std) :
    if isinstance(mean,list) and isinstance(std,list):
        assert len(mean)==len(std);
        return [get_sample(mean[i],std[i]) for i in range(len(mean))];
    else:
        return random.gauss(mean,std)

# Given ND samples, computes mean.
def get_mean(samps) :
    if isinstance(samps[0],list):
        # accumulate 
        cols = [[s[i] for s in samps] for i in range(len(samps[0]))];
        return [get_mean(col[1]) for col in enumerate(cols)]
    else:
        return sum(samps)/float(len(samps));


# Given 1D samples and mean, computes variance.
def get_variance(samps, mean):
    if isinstance(samps[0],list):
        cols = [[s[i] for s in samps] for i in range(len(samps[0]))];
        return [get_variance(col[1],mean[col[0]]) for col in enumerate(cols)]
    else:
        return sum((mean - value) ** 2 for value in samps) / len(samps)

# Main routine
def test1d(quantile=0.5, kld_error = 0.1, bin_size = 0.1, min_samples=10, seed=-1, umean=0, uvar=1):

  assert (quantile >= 0.5 and quantile <= 1), \
             "quantile must be between 0.5 and 1.0 - quantile is max thresholded at 0.99998"
  quantile=min(0.99998,quantile);

  assert (min_samples>=10), "min-samples needs to be at least 10."
  assert (kld_error  > 0), "kld_error must be greater than 0."
  assert (uvar > 0) ,  "underlying-var must be positive."
  assert (bin_size > 0) , "bin-size must be greater than 0."

  if (seed == -1):
    random.seed()
  else:
    random.seed(seed)

  print "Source distribution: 1D Gaussian with mean=" ,umean, " and variance=" , uvar;
  print "KLD quantile: " , quantile;
  print "KLD error: " , kld_error;
  print "KLD bin size: " , bin_size;
  print "Minimum # of samples: " , min_samples;
  print "Random Seed: ",seed,"\n";

  bins = [bin_size];

  sampler= KLDSampling(quantile,kld_error,bins,min_samples);

  num_samples=0;
  samples = [];

  while (num_samples < min_samples) :
    curr_sample = [[get_sample(umean,math.sqrt(uvar))]];

    samples.append(curr_sample);
    num_samples = num_samples+1;

    #make the sample into a 1D vector because the kld_sampling module
    #assumes multivariate distributions.
    #curr_sample2[0]=curr_sample;

    min_samples=sampler.update(curr_sample);

  sampleslist = [i[0] for i in samples]
  mean=get_mean(sampleslist);
  variance=get_variance(sampleslist,mean);

  print "Final number of samples: ",num_samples;
  print "Final mean: ",mean;
  print "Final variance: ",variance,"\n";


# Main routine
def test2d(quantile=0.5, kld_error = 0.1, bin_size = [0.1,0.1], min_samples=10, seed=-1, umean=[0,0], uvar=[1,1]):

  assert (quantile >= 0.5 and quantile <= 1), \
             "quantile must be between 0.5 and 1.0 - quantile is max thresholded at 0.99998"
  quantile=min(0.99998,quantile);

  assert (min_samples>=10), "min-samples needs to be at least 10."
  assert (kld_error  > 0), "kld_error must be greater than 0."
  assert (uvar[0]>0 and uvar[1]>0) ,  "underlying-var must be positive."
  assert (bin_size[0]>0 and bin_size[1]>0) , "bin-size must be greater than 0."

  if (seed == -1):
    random.seed()
  else:
    random.seed(seed)

  print "Source distribution: 2D Gaussian with mean=(",umean,") and variance=(" , uvar,")";
  print "KLD quantile: " , quantile;
  print "KLD error: " , kld_error;
  print "KLD bin size: " , bin_size;
  print "Minimum # of samples: " , min_samples;
  print "Random Seed: ",seed,"\n";

  sampler= KLDSampling(quantile,kld_error,bin_size,min_samples);

  num_samples=0;
  samples = [];
  ustd = [ math.sqrt(v) for v in uvar];

  while (num_samples < min_samples) :
    curr_sample = [get_sample(umean,ustd)];
    samples.append(curr_sample);
    num_samples = num_samples+1;
    min_samples=sampler.update(curr_sample);

  sampleslist = [i[0] for i in samples]
  mean=get_mean(sampleslist);
  variance=get_variance(sampleslist,mean);

  print "Final number of samples: ",num_samples;
  print "Final mean: ",mean;
  print "Final variance: ",variance,"\n";

if __name__ == '__main__':
  test1d ()
  test2d ()