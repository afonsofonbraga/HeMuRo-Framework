#!/bin/sh

#  run_linux.sh
#  MRSFramework
#
#  Created by Afonso Braga on 26/03/21.
#  Copyright © 2021 Afonso Braga. All rights reserved.

cd build
cmake ..
make
cd ..
./build/devel/lib/framework/framework --docroot . --http-address 0.0.0.0 --http-port 8080 --resources-dir=/usr/local/share/Wt/resources
