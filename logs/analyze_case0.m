load('stat_case0')

nodes.baseline = l1_baseline.nodes;
nodes.moprim = l2_moprim.nodes;
nodes.sparse_disturb = l3_sparse_disturb.nodes;
nodes.baseline = l5_sparse_nodisturb.nodes;
nodes.moprim = l6_sparse_disturb.nodes;
nodes.sparse_disturb = l7_sparse_disturb_moprim.nodes;

iter.baseline = l1_baseline.iter;
iter.moprim = l2_moprim.iter;
iter.sparse_disturb = l3_sparse_disturb.iter;
iter.baseline = l5_sparse_nodisturb.iter;
iter.moprim = l6_sparse_disturb.iter;
iter.sparse_disturb = l7_sparse_disturb_moprim.iter;

iter_change.baseline = l1_baseline.iter_change;
iter_change.moprim = l2_moprim.iter_change;
iter_change.sparse_disturb = l3_sparse_disturb.iter_change;
iter_change.baseline = l5_sparse_nodisturb.iter_change;
iter_change.moprim = l6_sparse_disturb.iter_change;
iter_change.sparse_disturb = l7_sparse_disturb_moprim.iter_change;

%% sparseness

sparse_nodes.l1 = l1_baseline.nodes;
sparse_nodes.l5 = l5_sparse_nodisturb.nodes;

sparse_iter.l1 = l1_baseline.iter;
sparse_iter.l5 = l5_sparse_nodisturb.iter;

sparse_iter_change.l1 = l1_baseline.iter_change;
sparse_iter_change.l5 = l5_sparse_nodisturb.iter_change;

sparse_iter_collide.l1 = l1_baseline.iter_collide;
sparse_iter_collide.l5 = l5_sparse_nodisturb.iter_collide;

sparse_pruned.l1 = l1_baseline.pruned;
sparse_pruned.l5 = l5_sparse_nodisturb.pruned;

sparse_witnesses.l1 = l1_baseline.witness;
sparse_witnesses.l5 = l5_sparse_nodisturb.witness;

nhist( sparse_nodes, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Histogram')
xlabel("Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline (s=0.0001,v=0.0002,delta=0.03)","sparse (s=0.07,v=0.12,delta=0.06)")

nhist( sparse_iter, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Total Histogram')
xlabel("Iterations Total")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline (s=0.0001,v=0.0002,delta=0.03)","sparse (s=0.07,v=0.12,delta=0.06)")

nhist( sparse_iter_change, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Changed Histogram')
xlabel("Iterations Changed")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline (s=0.0001,v=0.0002,delta=0.03)","sparse (s=0.07,v=0.12,delta=0.06)")

nhist( sparse_iter_collide, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Collide Histogram')
xlabel("Iterations Collide")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline (s=0.0001,v=0.0002,delta=0.03)","sparse (s=0.07,v=0.12,delta=0.06)")

nhist( sparse_pruned, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Pruned Histogram')
xlabel("Nodes Pruned")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline (s=0.0001,v=0.0002,delta=0.03)","sparse (s=0.07,v=0.12,delta=0.06)")

nhist( sparse_witnesses, 'proportion', 'samebins', 'binfactor', 4 );
title('Witnesses Histogram')
xlabel("Witnesses")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline (s=0.0001,v=0.0002,delta=0.03)","sparse (s=0.07,v=0.12,delta=0.06)")


%% motion primitives

moprim_nodes.l1 = l1_baseline.nodes;
moprim_nodes.l2 = l2_moprim.nodes;

moprim_iter.l1 = l1_baseline.iter;
moprim_iter.l2 = l2_moprim.iter;

moprim_iter_change.l1 = l1_baseline.iter_change;
moprim_iter_change.l2 = l2_moprim.iter_change;

moprim_iter_collide.l1 = l1_baseline.iter_collide;
moprim_iter_collide.l2 = l2_moprim.iter_collide;

nhist( moprim_nodes, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Histogram')
xlabel("Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","motion primitives")

nhist( moprim_iter, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Total Histogram')
xlabel("Iterations Total")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","motion primitives")

nhist( moprim_iter_change, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Changed Histogram')
xlabel("Iterations Changed")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","motion primitives")

nhist( moprim_iter_collide, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Collide Histogram')
xlabel("Iterations Collide")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("baseline","motion primitives")

%% witness disturbance

%disturb_nodes.l1 = l1_baseline.nodes;
disturb_nodes.l5 = l5_sparse_nodisturb.nodes;
disturb_nodes.l6 = l6_sparse_disturb.nodes;

%disturb_iter.l1 = l1_baseline.iter;
disturb_iter.l5 = l5_sparse_nodisturb.iter;
disturb_iter.l6 = l6_sparse_disturb.iter;

%disturb_iter_change.l1 = l1_baseline.iter_change;
disturb_iter_change.l5 = l5_sparse_nodisturb.iter_change;
disturb_iter_change.l6 = l6_sparse_disturb.iter_change;

%disturb_iter_collide.l1 = l1_baseline.iter_collide;
disturb_iter_collide.l5 = l5_sparse_nodisturb.iter_collide;
disturb_iter_collide.l6 = l6_sparse_disturb.iter_collide;

%disturb_pruned.l1 = l1_baseline.pruned;
disturb_pruned.l5 = l5_sparse_nodisturb.pruned;
disturb_pruned.l6 = l6_sparse_disturb.pruned;

%disturb_witnesses.l1 = l1_baseline.witness;
disturb_witnesses.l5 = l5_sparse_nodisturb.witness;
disturb_witnesses.l6 = l6_sparse_disturb.witness;

nhist( disturb_nodes, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Histogram')
xlabel("Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb")

nhist( disturb_iter, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Total Histogram')
xlabel("Iterations Total")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb")

nhist( disturb_iter_change, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Changed Histogram')
xlabel("Iterations Changed")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb")

nhist( disturb_iter_collide, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Collide Histogram')
xlabel("Iterations Collide")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb")

nhist( disturb_pruned, 'proportion', 'samebins', 'binfactor', 4 );
title('Nodes Pruned Histogram')
xlabel("Nodes Pruned")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb")

nhist( disturb_witnesses, 'proportion', 'samebins', 'binfactor', 4 );
title('Witnesses Histogram')
xlabel("Witnesses")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb")

%% witness disturbance + moprim
 
% disturb_moprim_nodes.l1 = l1_baseline.nodes;
disturb_moprim_nodes.l5 = l5_sparse_nodisturb.nodes;
disturb_moprim_nodes.l6 = l6_sparse_disturb.nodes;
disturb_moprim_nodes.l7 = l7_sparse_disturb_moprim.nodes;

% disturb_moprim_iter.l1 = l1_baseline.iter;
disturb_moprim_iter.l5 = l5_sparse_nodisturb.iter;
disturb_moprim_iter.l6 = l6_sparse_disturb.iter;
disturb_moprim_iter.l7 = l7_sparse_disturb_moprim.iter;

% disturb_moprim_iter_change.l1 = l1_baseline.iter_change;
disturb_moprim_iter_change.l5 = l5_sparse_nodisturb.iter_change;
disturb_moprim_iter_change.l6 = l6_sparse_disturb.iter_change;
disturb_moprim_iter_change.l7 = l7_sparse_disturb_moprim.iter_change;

% disturb_moprim_iter_collide.l1 = l1_baseline.iter_collide;
disturb_moprim_iter_collide.l5 = l5_sparse_nodisturb.iter_collide;
disturb_moprim_iter_collide.l6 = l6_sparse_disturb.iter_collide;
disturb_moprim_iter_collide.l7 = l7_sparse_disturb_moprim.iter_collide;

% disturb_moprim_witnesses.l1 = l1_baseline.witness;
disturb_moprim_witnesses.l5 = l5_sparse_nodisturb.witness;
disturb_moprim_witnesses.l6 = l6_sparse_disturb.witness;
disturb_moprim_witnesses.l7 = l7_sparse_disturb_moprim.witness;

% disturb_moprim_moprim_invoked.l1 = l1_baseline.moprim;
% disturb_moprim_moprim_invoked.l5 = l5_sparse_nodisturb.moprim;
% disturb_moprim_moprim_invoked.l6 = l6_sparse_disturb.moprim;
disturb_moprim_moprim_invoked.l7 = l7_sparse_disturb_moprim.moprim;

nhist( disturb_moprim_nodes, 'proportion', 'samebins', 'binfactor', 3 );
title('Nodes Histogram')
xlabel("Nodes")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb","sparse+disturb+moprim")

nhist( disturb_moprim_iter, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Total Histogram')
xlabel("Iterations Total")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb","sparse+disturb+moprim")

nhist( disturb_moprim_iter_change, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Changed Histogram')
xlabel("Iterations Changed")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb","sparse+disturb+moprim")

nhist( disturb_moprim_iter_collide, 'proportion', 'samebins', 'binfactor', 4 );
title('Iterations Collide Histogram')
xlabel("Iterations Collide")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse","sparse+disturb","sparse+disturb+moprim")

nhist( disturb_moprim_moprim_invoked, 'proportion', 'samebins', 'binfactor', 4 );
title('Motion Primitives Invoked Histogram')
xlabel("Motion Primitives Invoked")
ylabel("Percentage")
ytix = get(gca, 'YTick')
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend("sparse+disturb+moprim")
