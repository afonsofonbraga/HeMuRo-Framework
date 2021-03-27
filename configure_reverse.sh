#!/bin/sh

#  configure_reverse.sh
#  MRSFramework
#
#  Created by Afonso Braga on 26/03/21.
#  Copyright © 2021 Afonso Braga. All rights reserved.

grep -rl '/Users/afonsofonbraga/github/HeMuRo-Framework/' ./src | xargs sed -i ''  's+/Users/afonsofonbraga/github/HeMuRo-Framework/+$path_to_HeMuRo/+g'
