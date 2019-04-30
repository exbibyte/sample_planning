load('stat_obs3d_airplane')

nhist( stat_obs3d_airplane.nodes, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Histogram')
xlabel("Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)

nhist( stat_obs3d_airplane.iter, 'proportion', 'samebins', 'binfactor', 1 );
title('Total Iterations Histogram')
xlabel("Total Iterations")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)