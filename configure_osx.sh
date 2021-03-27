#!/bin/sh

#  configure_MacOS.sh
#  MRSFramework
#
#  Created by Afonso Braga on 26/03/21.
#  Copyright © 2021 Afonso Braga. All rights reserved.

#path =  std::string("path_to_HeMuRo/logs/Logger")
grep -rl '$path_to_HeMuRo/' ./src | xargs sed -i ''  's+$path_to_HeMuRo/+/Users/afonsofonbraga/github/HeMuRo-Framework/+g'

