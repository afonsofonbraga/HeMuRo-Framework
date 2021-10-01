#!/bin/sh

#  configure_reverse.sh
#  MRSFramework
#
#  Created by Afonso Braga on 26/03/21.
#  Copyright © 2021 Afonso Braga. All rights reserved.

grep -rl '/Users/afonsofonbraga/github/HeMuRo-Framework/' ./src | xargs sed -i ''  's+/Users/afonsofonbraga/github/HeMuRo-Framework/+$path_to_HeMuRo/+g'
#grep -rl '/Users/afonso/Github/MRSFramework/' ./src | xargs sed -i ''  's+/Users/afonso/Github/MRSFramework/+$path_to_HeMuRo/+g'

#grep -rl '/home/robot/github/HeMuRo-Framework/' ./src | xargs sed -i 's+/home/robot/github/HeMuRo-Framework/+$path_to_HeMuRo/+g'
#grep -rl '/home/clever/Github/MRSFramework/' ./src | xargs sed -i 's+/home/clever/Github/MRSFramework/+$path_to_HeMuRo/+g'
