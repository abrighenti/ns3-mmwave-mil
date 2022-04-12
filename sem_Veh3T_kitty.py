def main():

    import sem
    import numpy as np
    import pandas as pd
    import os
    import matplotlib.pyplot as plt

    # Define campaign parameters
    ############################

    script = 'vehicular-threeTransmitters_2plat_sem'
    cmp_name = 'sem_3Transmitters_kitty_first' # define a campaign name in order to create different folders corresponding to a specific configuration
    ns_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    campaign_dir = "./campaigns/"+cmp_name

    # Create campaign
    #################

    campaign = sem.CampaignManager.new(ns_path, script, campaign_dir,
        runner_type='ParallelRunner',
        check_repo=False,
        overwrite=False)

    # Parameter space
    #################

    '''
    interGroupDist_list=[10, 50, 80]
    intraGroupDist_list=[20, 40, 80]
    csma_list=[True, False]
    antennaElements_list=[4, 16]
    rngRun_list=[2*j+1 for j in range(0,30)]
    threshold_list=[0.0, 1e-15, 1.0]
    '''
    interGroupDist_list=[10, 80]
    intraGroupDist_list=[20, 80]
    csma_list=[True, False]
    antennaElements_list=[16]
    rngRun_list=[2*j+1 for j in range(0,20)]
    threshold_list=[0.0, 1e-15, 1.0]
    

    param_combinations = {
        'CSMA': csma_list,
        'interGroupDistance': interGroupDist_list,
        'intraGroupDistance': intraGroupDist_list,
        'numAntennaElements': antennaElements_list,
        'RngRun': rngRun_list,
        'threshold': threshold_list
    }

    # Run simulations
    #################


    campaign.run_missing_simulations(sem.list_param_combinations(param_combinations), stop_on_errors=False)

    print("Simulations done.")

if __name__ == '__main__':
    main()