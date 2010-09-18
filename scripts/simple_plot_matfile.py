import numpy as np
import scipy.io
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import sys

# column indices in the trajectories array
OBJ_ID_IDX=0
TSTAMP_IDX=1
X_IDX=2
Y_IDX=3

if 1:
    if len(sys.argv) != 3:
        print >> sys.stderr, """Invalid arguments. Usage:

  python simple_plot_matfile.py <data.mat> <outfile.pdf>

  Where the following names have been replaced with appropriate filenames:

  <data.mat>     Input .mat file saved by 'convert_to_matfile'
  <outfile.pdf>  Output .pdf file with trajectories plotted
  """
        sys.exit(1)

    fname = sys.argv[1]
    pdf_fname = sys.argv[2]
    data = scipy.io.loadmat(fname,struct_as_record=True)

    trials = data['trials']
    trajectories = data['trajectories']

    pdf_pages = PdfPages(pdf_fname)

    for trial in trials:
        tstamp, obj_id, magnet_obj_id, condition = trial
        assert condition==0

        fly_cond = trajectories[:,OBJ_ID_IDX]==obj_id
        magnet_cond = trajectories[:,OBJ_ID_IDX]==magnet_obj_id
        tcond = (trajectories[:,TSTAMP_IDX]>=(tstamp-1.5)) & \
                (trajectories[:,TSTAMP_IDX]<=(tstamp+1.5))

        joint_cond = fly_cond & tcond

        fly_data = trajectories[ fly_cond & tcond ]
        fly_time = fly_data[:,TSTAMP_IDX]
        fly_x = fly_data[:,X_IDX]
        fly_y = fly_data[:,Y_IDX]

        magnet_data = trajectories[ magnet_cond & tcond ]
        magnet_time = magnet_data[:,TSTAMP_IDX]
        magnet_x = magnet_data[:,X_IDX]
        magnet_y = magnet_data[:,Y_IDX]

        if len(fly_x)==0:
            print >> sys.stderr, ('ERROR: No data at %s for fly %d, not plotting.'%
                                  (tstamp,obj_id,))
            continue

        if len(magnet_x)==0:
            print >> sys.stderr, ('ERROR: No data at %s for magnet %d, not plotting.'%
                                  (tstamp,magnet_obj_id,))
            continue

        fig = plt.figure()

        ax = fig.add_subplot(3,1,1)
        ax.set_aspect('equal')
        ax.plot( fly_x, fly_y, 'x-', label='fly %d'%obj_id )
        ax.plot( magnet_x, magnet_y, 'o-', label='magnet %d'%magnet_obj_id )
        ax.set_xlabel('x (pix)')
        ax.set_ylabel('y (pix)')

        ax2 = fig.add_subplot(3,1,2)
        ax2.plot( fly_time-tstamp, fly_x, 'x-', label='fly %d'%obj_id )
        ax2.plot( magnet_time-tstamp, magnet_x, 'o-', label='magnet %d'%magnet_obj_id )
        ax2.set_xlabel('t (sec)')
        ax2.set_ylabel('x (pix)')

        ax3 = fig.add_subplot(3,1,3, sharex=ax2)
        ax3.plot( fly_time-tstamp, fly_y, 'x-', label='fly %d'%obj_id )
        ax3.plot( magnet_time-tstamp, magnet_y, 'o-', label='magnet %d'%magnet_obj_id )
        ax3.set_xlabel('t (sec)')
        ax3.set_ylabel('y (pix)')

        ax3.legend()

        fig.subplots_adjust(hspace=0.39)
        pdf_pages.savefig(fig)

        fig.clf()
        del fig
    pdf_pages.close()
