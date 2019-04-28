load('stat_case1')

nodes.baseline = baseline.nodes;
nodes.frontier = frontier.nodes;
nodes.control = control.nodes;
nodes.frontiercontrol = frontiercontrol.nodes;

witness.baseline = baseline.witness;
witness.frontier = frontier.witness;
witness.control = control.witness;
witness.frontiercontrol = frontiercontrol.witness;

pruned.baseline = baseline.pruned;
pruned.frontier = frontier.pruned;
pruned.control = control.pruned;
pruned.frontiercontrol = frontiercontrol.pruned;

iter_change.baseline = baseline.iter_change;
iter_change.frontier = frontier.iter_change;
iter_change.control = control.iter_change;
iter_change.frontiercontrol = frontiercontrol.iter_change;

nhist( nodes, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Histogram')
xlabel("Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","frontier node", "control sel", "frontier node + control sel")

nhist( witness, 'proportion', 'samebins', 'binfactor', 4 );
title('Witnesses Histogram')
xlabel("Witnesses")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","frontier node", "control sel", "frontier node + control sel")

nhist( pruned, 'proportion', 'samebins', 'binfactor', 4 );
title('Pruned Nodes Histogram')
xlabel("Pruned Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","frontier node", "control sel", "frontier node + control sel")

nhist( iter_change, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Changed Histogram')
xlabel("Iterations Changed")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","frontier node", "control sel", "frontier node + control sel")

