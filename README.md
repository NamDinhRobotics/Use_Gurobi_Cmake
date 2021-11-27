# Install the Gurobi Optimizer and test it

### Step 1:
Download gurobi9.5.0_linux64.tar.gz from _https://www.gurobi.com/downloads/download-center_.

### Step 2:
You can read more detail in
[quickstart_linux](https://cdn.gurobi.com/wp-content/plugins/hd_documentations/documentation/9.0/quickstart_linux.pdf)

Your next step is to choose a destination directory.\
I recommend _/opt_ for a shared installation, but other directories will work as well.

Copy the Gurobi distribution to the destination directory
and extract the contents.

Extraction is done with the following command:\
`tar xvfz gurobi9.0.1_linux64.tar.gz`

This command will create a sub-directory _/opt/gurobi950/linux64_ that contains the complete Gurobi distribution (assuming you chose /opt).

Your <installdir> (which we’ll refer to throughout this document) will be _/opt/gurobi950/linux64_.

The Gurobi Optimizer makes use of several executable files. In order to allow these files to be found when needed, you will have to modify a few environment variables:
+ GUROBI_HOME should point to your <installdir>.
+ PATH should be extended to include <installdir>/bin.
+ LD_LIBRARY_PATH should be extended to include <installdir>/lib.

Users of the bash shell should add the following lines to their .bashrc files:

`export GUROBI_HOME="/opt/gurobi901/linux64"`\
`export PATH="${PATH}:${GUROBI_HOME}/bin"`\
`export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"`

### Step 3: You can use academic license as follows
Please check this link for more detail\
_https://www.youtube.com/watch?v=fRKhao2bzsY&t=373s_


### Step 4:

Copy our _CmakeLists.txt_ file to the <installdir>_/src_ directory.

And _FindGUROBI.cmake_ file to the <installdir>_/src_ directory.

###### Then, enjoy the Gurobi Optimizer!


###You can test your installation typing gurobi.sh in the terminal

Issues when installing Gurobi:\
If you find the error:>\
``“gurobi_continuous.cpp:(.text.startup+0x74): undefined reference to
`GRBModel::set(GRB_StringAttr, std::__cxx11::basic_string<char,
std::char_traits<char>, std::allocator<char> > const&)'”
``

The solution is:\
`cd /opt/gurobi950/linux64/src/build ` \
Note that the name of the folder gurobi950 changes according to the Gurobi version \
`sudo make` \
`sudo cp libgurobi_c++.a ../../lib/`

####References:
[FASTER planing from MIT](https://github.com/mit-acl/faster)
