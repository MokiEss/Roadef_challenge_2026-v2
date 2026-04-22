// ============================================================================
// T-ASR Visualizer — Main Application
// ROADEF/EURO 2026 Challenge Interactive Visualization
// ============================================================================

(function () {
    'use strict';

    // ========================================================================
    // STATE
    // ========================================================================
    const State = {
        data: null,           // FullSolverState
        currentSnapshotIdx: 0,
        currentTimeSlot: 0,
        selectedDemandId: null,
        selectedArcId: null,
        cy: null,             // Cytoscape instance
        highlightedRoute: [],
        showLabels: true,
        showMlu: true,
        showDistance: false,
        colorByLoad: true,
        // Waypoint editing
        editingWaypoints: null,     // array of node IDs being edited, or null
        originalWaypoints: null,    // original waypoints before editing
        graph: null,                // adjacency list for Dijkstra
        addWaypointMode: false,     // clicking a node adds it as waypoint
        currentLayout: 'fcose',     // layout algorithm
        showingFlow: false,         // flow overlay mode active
        flowDemandId: null,         // demand whose flow is shown
        // Multi-demand selection
        selectedDemandIds: new Set(), // checked demands for multi-route view
        showingMultiRoutes: false,    // multi-route overlay active
        // Demand table sorting
        demandSortKey: 'id',          // 'id' | 'source' | 'target' | 'volume'
        demandSortAsc: true,
        noRevisit: false,             // prevent revisiting nodes across ECMP segments
        arcSortKey: 'load',           // 'id' | 'from' | 'to' | 'load'
        arcSortAsc: false,            // default: descending by load
    };

    // ========================================================================
    // HELPERS
    // ========================================================================
    function fmt(val, decimals = 4) {
        if (val == null || val === undefined) return '—';
        if (typeof val !== 'number' || !isFinite(val)) return '∞';
        return val.toFixed(decimals);
    }

    function getLoadColor(load) {
        if (load > 1.0) return '#f7768e';       // overloaded - red
        if (load > 0.8) return '#ff9e64';       // high - orange
        if (load > 0.5) return '#e0af68';       // medium - yellow
        if (load > 0.2) return '#7aa2f7';       // normal - blue
        return '#565f89';                        // low - muted
    }

    function getLoadClass(load) {
        if (load > 0.9) return 'danger';
        if (load > 0.7) return 'warning';
        return 'good';
    }

    function getCurrentSnapshot() {
        if (!State.data || !State.data.snapshots.length) return null;
        // Find snapshot matching current time slot at closest iteration
        const matching = State.data.snapshots.filter(s => s.time === State.currentTimeSlot);
        if (matching.length > 0) {
            const idx = Math.min(State.currentSnapshotIdx, matching.length - 1);
            return matching[idx];
        }
        return State.data.snapshots[State.currentSnapshotIdx] || null;
    }

    function getSnapshotsForTime(t) {
        if (!State.data) return [];
        return State.data.snapshots.filter(s => s.time === t);
    }

    function getAllTimeSlots() {
        if (!State.data) return [];
        const times = new Set(State.data.snapshots.map(s => s.time));
        return Array.from(times).sort((a, b) => a - b);
    }

    // ========================================================================
    // DATA LOADING
    // ========================================================================
    function loadData(jsonData) {
        State.data = jsonData;
        State.currentSnapshotIdx = 0;
        State.currentTimeSlot = jsonData.snapshots.length > 0 ? jsonData.snapshots[0].time : 0;
        State.selectedDemandId = null;
        State.selectedArcId = null;
        State.editingWaypoints = null;
        State.originalWaypoints = null;
        State.selectedDemandIds = new Set();
        State.showingMultiRoutes = false;

        updateInstanceInfo();
        buildGraph();
        initGraph();
        updateAll();
    }

    // ========================================================================
    // ROUTE COLOR PALETTE
    // ========================================================================
    const ROUTE_COLORS = [
        '#7aa2f7', // blue
        '#9ece6a', // green
        '#ff9e64', // orange
        '#bb9af7', // purple
        '#f7768e', // red/pink
        '#73daca', // teal
        '#e0af68', // yellow
        '#2ac3de', // cyan
        '#ff007c', // magenta
        '#c3e88d', // lime
        '#82aaff', // light blue
        '#c792ea', // lavender
    ];

    function getRouteColor(index) {
        return ROUTE_COLORS[index % ROUTE_COLORS.length];
    }

    // ========================================================================
    // INSTANCE FILE LOADING
    // ========================================================================

    function readFileAsJson(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = function (ev) {
                try { resolve(JSON.parse(ev.target.result)); }
                catch (e) { reject(new Error('Invalid JSON in ' + file.name + ': ' + e.message)); }
            };
            reader.onerror = function () { reject(new Error('Could not read ' + file.name)); };
            reader.readAsText(file);
        });
    }

    async function loadFromInstanceFiles(netFile, tmFile, scenarioFile, solutionFile) {
        const errorEl = document.getElementById('inst-load-error');
        errorEl.classList.add('hidden');

        try {
            const promises = [readFileAsJson(netFile), readFileAsJson(tmFile), readFileAsJson(scenarioFile)];
            if (solutionFile) promises.push(readFileAsJson(solutionFile));

            const results = await Promise.all(promises);
            const netData = results[0];
            const tmData = results[1];
            const scenarioData = results[2];
            const solutionData = results[3] || null;

            // Build solver state from raw instance files
            const solverState = buildSolverStateFromInstance(netData, tmData, scenarioData, solutionData);
            loadData(solverState);

            // Close modal
            document.getElementById('instance-modal').classList.add('hidden');
        } catch (err) {
            errorEl.textContent = err.message;
            errorEl.classList.remove('hidden');
        }
    }

    function buildSolverStateFromInstance(netData, tmData, scenarioData, solutionData) {
        // Parse nodes
        const nodes = (netData.nodes || []).map(n => ({
            id: typeof n.id === 'number' ? n.id : parseInt(n.id),
            name: n.name || ('v' + n.id),
        }));

        // Parse arcs (links)
        const arcs = (netData.links || []).map(l => ({
            id: typeof l.id === 'number' ? l.id : parseInt(l.id),
            from: typeof l.from === 'number' ? l.from : parseInt(l.from),
            to: typeof l.to === 'number' ? l.to : parseInt(l.to),
            metric: l.metric || 1,
            capacity: l.capacity || 1,
        }));

        const numTimeSlots = tmData.num_time_slots || 1;
        const maxSegments = scenarioData.max_segments || 6;
        const numNodes = nodes.length;
        const numArcs = arcs.length;
        const numDemands = (tmData.demands || []).length;

        // Build waypoint lookup from solution: key = "d_t" -> waypoints array
        const waypointMap = {};
        if (solutionData && solutionData.srpaths) {
            solutionData.srpaths.forEach(sp => {
                waypointMap[sp.d + '_' + sp.t] = sp.w || [];
            });
        }

        // Build adjacency list for Dijkstra
        const graph = {};
        nodes.forEach(n => { graph[n.id] = []; });
        arcs.forEach(a => {
            graph[a.from].push({ to: a.to, arcId: a.id, metric: a.metric });
        });

        // Dijkstra (internal, does not use State.graph)
        function localDijkstra(source, target) {
            const dist = {};
            const prev = {};
            const prevArc = {};
            const visited = new Set();
            nodes.forEach(n => { dist[n.id] = Infinity; });
            dist[source] = 0;
            const pq = [{ node: source, d: 0 }];
            while (pq.length > 0) {
                pq.sort((a, b) => a.d - b.d);
                const { node: u, d: du } = pq.shift();
                if (visited.has(u)) continue;
                visited.add(u);
                if (u === target) break;
                if (!graph[u]) continue;
                for (const edge of graph[u]) {
                    const nd = du + edge.metric;
                    if (nd < dist[edge.to]) {
                        dist[edge.to] = nd;
                        prev[edge.to] = u;
                        prevArc[edge.to] = edge.arcId;
                        pq.push({ node: edge.to, d: nd });
                    }
                }
            }
            if (dist[target] === Infinity) return null;
            const arcList = [];
            let cur = target;
            while (cur !== source) { arcList.unshift(prevArc[cur]); cur = prev[cur]; }
            return arcList;
        }

        // Build budget lookup: t -> value
        const budgetMap = {};
        (scenarioData.budget || []).forEach(b => { budgetMap[b.t] = b.value; });

        // Build intervention lookup: t -> [link ids]
        const interventionMap = {};
        (scenarioData.interventions || []).forEach(iv => {
            interventionMap[iv.t] = iv.links || [];
        });

        // Create one snapshot per time slot
        const snapshots = [];

        for (let t = 0; t < numTimeSlots; t++) {
            const interventionArcs = interventionMap[t] || [];
            const interventionSet = new Set(interventionArcs);

            // Accumulate flows per arc
            const flowAccum = {};
            arcs.forEach(a => { flowAccum[a.id] = 0; });

            const demands = [];
            (tmData.demands || []).forEach((rawDemand, dIdx) => {
                const source = rawDemand.s;
                const target = rawDemand.t;
                const volume = (rawDemand.v && rawDemand.v[t] !== undefined) ? rawDemand.v[t] : 0;
                const waypoints = waypointMap[dIdx + '_' + t] || [];

                demands.push({
                    demand_id: dIdx,
                    source: source,
                    target: target,
                    volume: volume,
                    waypoints: waypoints,
                    contribution: 0,
                    is_candidate: false,
                });

                // Compute path via segment routing
                if (volume > 0) {
                    const segEndpoints = [source, ...waypoints, target];
                    for (let i = 0; i < segEndpoints.length - 1; i++) {
                        const pathArcs = localDijkstra(segEndpoints[i], segEndpoints[i + 1]);
                        if (pathArcs) {
                            pathArcs.forEach(arcId => {
                                if (!interventionSet.has(arcId)) {
                                    flowAccum[arcId] += volume;
                                }
                            });
                        }
                    }
                }
            });

            // Compute loads
            const arcLoads = [];
            let mlu = 0;
            let mostCongestedId = 0;
            arcs.forEach(a => {
                const flow = flowAccum[a.id] || 0;
                const cap = a.capacity || 1;
                const load = interventionSet.has(a.id) ? 0 : flow / cap;
                if (load > mlu) { mlu = load; mostCongestedId = a.id; }
                arcLoads.push({
                    arc_id: a.id,
                    from: a.from,
                    to: a.to,
                    load: load,
                    capacity: cap,
                    flow: flow,
                });
            });

            // Mark candidates on most congested arc
            if (mlu > 0) {
                const congestedArc = arcs.find(a => a.id === mostCongestedId);
                if (congestedArc) {
                    demands.forEach(d => {
                        if (d.volume > 0) {
                            const segEndpoints = [d.source, ...d.waypoints, d.target];
                            for (let i = 0; i < segEndpoints.length - 1; i++) {
                                const pathArcs = localDijkstra(segEndpoints[i], segEndpoints[i + 1]);
                                if (pathArcs && pathArcs.includes(mostCongestedId)) {
                                    d.is_candidate = true;
                                    d.contribution = d.volume;
                                    break;
                                }
                            }
                        }
                    });
                }
            }

            const topLoads = arcLoads.map(a => a.load).sort((a, b) => b - a).slice(0, 10);
            const budgetAllowed = budgetMap[t] || 0;

            snapshots.push({
                iteration: 0,
                time: t,
                objective: { mlu: mlu, top_loads: topLoads },
                budget: { used: 0, allowed: budgetAllowed },
                violations: [],
                intervention_arcs: interventionArcs,
                most_congested_arc_id: mostCongestedId,
                arc_loads: arcLoads,
                demands: demands,
                move: { type: 'initial', demand_id: -1, details: 'Instance loaded (t=' + t + ')' },
            });
        }

        // Derive instance name from file names
        const instanceName = 'instance';

        return {
            instance_name: instanceName,
            num_nodes: numNodes,
            num_arcs: numArcs,
            num_demands: numDemands,
            num_time_slots: numTimeSlots,
            max_segments: maxSegments,
            nodes: nodes,
            arcs: arcs,
            snapshots: snapshots,
        };
    }

    function updateInstanceInfo() {
        const d = State.data;
        const info = `${d.instance_name} — ${d.num_nodes} nodes, ${d.num_arcs} arcs, ${d.num_demands} demands, ${d.num_time_slots} time slots, maxSeg=${d.max_segments}`;
        document.getElementById('instance-info').textContent = info;
    }

    // ========================================================================
    // CYTOSCAPE GRAPH
    // ========================================================================
    function getLayoutConfig(name) {
        const metricEdgeLength = function (edge) {
            const m = edge.data('metric') || 100;
            return Math.max(120, Math.min(500, m * 1.2));
        };
        switch (name) {
            case 'fcose':
                return {
                    name: 'fcose',
                    animate: false,
                    quality: 'proof',
                    randomize: true,
                    nodeRepulsion: function () { return 800000; },
                    idealEdgeLength: metricEdgeLength,
                    edgeElasticity: function () { return 0.1; },
                    gravity: 0.02,
                    gravityRange: 3.8,
                    numIter: 10000,
                    padding: 100,
                    nodeSeparation: 120,
                    piTol: 0.0000001,
                    packComponents: true,
                    sampleSize: 50,
                    tile: true,
                    tilingPaddingVertical: 30,
                    tilingPaddingHorizontal: 30,
                };
            case 'cose':
                return {
                    name: 'cose',
                    animate: false,
                    nodeRepulsion: function () { return 500000; },
                    idealEdgeLength: metricEdgeLength,
                    edgeElasticity: function () { return 50; },
                    gravity: 0.04,
                    numIter: 5000,
                    padding: 80,
                    nodeOverlap: 80,
                    componentSpacing: 400,
                    nestingFactor: 1.2,
                    randomize: false,
                };
            case 'circle':
                return { name: 'circle', animate: false, padding: 60, spacingFactor: 1.5 };
            case 'grid':
                return { name: 'grid', animate: false, padding: 60, spacingFactor: 1.2, condense: false };
            case 'concentric':
                return {
                    name: 'concentric',
                    animate: false,
                    padding: 60,
                    spacingFactor: 1.5,
                    concentric: function (node) {
                        return node.degree();
                    },
                    levelWidth: function () { return 2; },
                };
            default:
                return { name: 'fcose', animate: false, quality: 'proof', randomize: true };
        }
    }

    function initGraph() {
        const d = State.data;
        const elements = [];

        // Add nodes
        d.nodes.forEach(n => {
            elements.push({
                data: { id: 'n' + n.id, label: n.name, nodeId: n.id },
                classes: 'network-node'
            });
        });

        // Add arcs
        d.arcs.forEach(a => {
            elements.push({
                data: {
                    id: 'e' + a.id,
                    source: 'n' + a.from,
                    target: 'n' + a.to,
                    arcId: a.id,
                    metric: a.metric,
                    capacity: a.capacity,
                    label: '',
                    load: 0,
                },
                classes: 'network-edge'
            });
        });

        if (State.cy) State.cy.destroy();

        State.cy = cytoscape({
            container: document.getElementById('cy'),
            elements: elements,
            style: getCytoscapeStyle(),
            layout: getLayoutConfig(State.currentLayout),
            minZoom: 0.1,
            maxZoom: 5,
            wheelSensitivity: 0.3,
        });

        // Event handlers
        State.cy.on('tap', 'edge', function (evt) {
            const arcId = evt.target.data('arcId');
            selectArc(arcId);
        });

        State.cy.on('tap', 'node', function (evt) {
            const nodeId = evt.target.data('nodeId');

            // If editing waypoints, clicking a node adds it as waypoint
            if (State.editingWaypoints !== null && State.selectedDemandId !== null) {
                const snap = getCurrentSnapshot();
                if (snap) {
                    const demand = snap.demands.find(d => d.demand_id === State.selectedDemandId);
                    if (demand && nodeId !== demand.source && nodeId !== demand.target) {
                        addWaypoint(demand, nodeId);
                        return;
                    }
                }
            }

            showNodeTooltip(evt, nodeId);
        });

        State.cy.on('mouseover', 'edge', function (evt) {
            showEdgeTooltip(evt);
        });

        State.cy.on('mouseout', 'edge', function () {
            hideTooltip();
        });

        State.cy.on('mouseover', 'node', function (evt) {
            showNodeHoverTooltip(evt);
        });

        State.cy.on('mouseout', 'node', function () {
            hideTooltip();
        });

        State.cy.on('tap', function (evt) {
            if (evt.target === State.cy) {
                clearSelection();
            }
        });
    }

    function getCytoscapeStyle() {
        return [
            {
                selector: 'node',
                style: {
                    'background-color': '#3b4261',
                    'border-color': '#7aa2f7',
                    'border-width': 1.5,
                    'label': State.showLabels ? 'data(label)' : '',
                    'color': '#c0caf5',
                    'font-size': 14,
                    'text-halign': 'center',
                    'text-valign': 'bottom',
                    'text-margin-y': 10,
                    'width': 36,
                    'height': 36,
                }
            },
            {
                selector: 'edge',
                style: {
                    'width': 2,
                    'line-color': '#565f89',
                    'target-arrow-color': '#565f89',
                    'target-arrow-shape': 'triangle',
                    'curve-style': 'bezier',
                    'arrow-scale': 0.6,
                    'label': (State.showMlu || State.showDistance) ? 'data(label)' : '',
                    'color': '#7982a9',
                    'font-size': 9,
                    'text-rotation': 'autorotate',
                    'text-margin-y': -10,
                    'text-background-color': '#1a1b26',
                    'text-background-opacity': 0.7,
                    'text-background-padding': 2,
                }
            },
            {
                selector: 'edge.overloaded',
                style: {
                    'line-color': '#f7768e',
                    'target-arrow-color': '#f7768e',
                    'width': 4,
                }
            },
            {
                selector: 'edge.high-load',
                style: {
                    'line-color': '#ff9e64',
                    'target-arrow-color': '#ff9e64',
                    'width': 3,
                }
            },
            {
                selector: 'edge.medium-load',
                style: {
                    'line-color': '#e0af68',
                    'target-arrow-color': '#e0af68',
                    'width': 2.5,
                }
            },
            {
                selector: 'edge.intervention',
                style: {
                    'line-style': 'dashed',
                    'line-color': '#f7768e',
                    'target-arrow-color': '#f7768e',
                    'line-dash-pattern': [6, 3],
                    'opacity': 0.6,
                }
            },
            {
                selector: 'edge.highlighted-route',
                style: {
                    'line-color': '#9ece6a',
                    'target-arrow-color': '#9ece6a',
                    'width': 4,
                    'z-index': 999,
                }
            },
            {
                selector: 'node.highlighted',
                style: {
                    'background-color': '#9ece6a',
                    'border-color': '#9ece6a',
                    'border-width': 3,
                    'width': 34,
                    'height': 34,
                }
            },
            {
                selector: 'node.source-node',
                style: {
                    'background-color': '#7aa2f7',
                    'border-color': '#7aa2f7',
                    'border-width': 3,
                    'width': 34,
                    'height': 34,
                }
            },
            {
                selector: 'node.target-node',
                style: {
                    'background-color': '#bb9af7',
                    'border-color': '#bb9af7',
                    'border-width': 3,
                    'width': 34,
                    'height': 34,
                }
            },
            {
                selector: 'edge.selected-arc',
                style: {
                    'line-color': '#bb9af7',
                    'target-arrow-color': '#bb9af7',
                    'width': 5,
                    'z-index': 999,
                }
            },
            {
                selector: 'edge.ecmp-path',
                style: {
                    'line-color': '#73daca',
                    'target-arrow-color': '#73daca',
                    'width': 3,
                    'z-index': 998,
                    'line-style': 'dashed',
                    'line-dash-pattern': [8, 4],
                }
            },
            {
                selector: 'edge.segment-0',
                style: { 'line-color': '#7aa2f7', 'target-arrow-color': '#7aa2f7', 'width': 4, 'z-index': 999 }
            },
            {
                selector: 'edge.segment-1',
                style: { 'line-color': '#9ece6a', 'target-arrow-color': '#9ece6a', 'width': 4, 'z-index': 999 }
            },
            {
                selector: 'edge.segment-2',
                style: { 'line-color': '#ff9e64', 'target-arrow-color': '#ff9e64', 'width': 4, 'z-index': 999 }
            },
            {
                selector: 'edge.segment-3',
                style: { 'line-color': '#e0af68', 'target-arrow-color': '#e0af68', 'width': 4, 'z-index': 999 }
            },
            {
                selector: 'edge.segment-4',
                style: { 'line-color': '#bb9af7', 'target-arrow-color': '#bb9af7', 'width': 4, 'z-index': 999 }
            },
            {
                selector: 'edge.segment-5',
                style: { 'line-color': '#f7768e', 'target-arrow-color': '#f7768e', 'width': 4, 'z-index': 999 }
            },
            {
                selector: 'node.waypoint-node',
                style: {
                    'background-color': '#ff9e64',
                    'border-color': '#ff9e64',
                    'border-width': 3,
                    'width': 40,
                    'height': 40,
                    'shape': 'diamond',
                    'label': 'data(label)',
                }
            },
            {
                selector: '.flow-dimmed',
                style: {
                    'opacity': 0.12,
                }
            },
            {
                selector: 'edge.flow-edge',
                style: {
                    'opacity': 1,
                    'z-index': 999,
                    'label': 'data(label)',
                    'font-size': 10,
                    'font-weight': 'bold',
                    'color': '#c0caf5',
                    'text-rotation': 'autorotate',
                    'text-margin-y': -12,
                    'text-background-color': '#1a1b26',
                    'text-background-opacity': 0.85,
                    'text-background-padding': 3,
                }
            },
            {
                selector: 'node.flow-node',
                style: {
                    'opacity': 1,
                    'background-color': '#3b4261',
                    'border-color': '#73daca',
                    'border-width': 2,
                }
            },
        ];
    }

    function updateGraphLoads() {
        const snap = getCurrentSnapshot();
        if (!snap || !State.cy) return;

        // Don't overwrite flow overlay or multi-route overlay
        if (State.showingFlow || State.showingMultiRoutes) return;

        // Build arc load map
        const loadMap = {};
        snap.arc_loads.forEach(al => { loadMap[al.arc_id] = al; });

        // Intervention set
        const interventionSet = new Set(snap.intervention_arcs || []);

        State.cy.edges().forEach(edge => {
            const arcId = edge.data('arcId');
            const al = loadMap[arcId];

            // Remove all dynamic classes
            edge.removeClass('overloaded high-load medium-load intervention highlighted-route selected-arc ecmp-path segment-0 segment-1 segment-2 segment-3 segment-4 segment-5');

            if (interventionSet.has(arcId)) {
                edge.addClass('intervention');
                edge.data('label', (State.showMlu || State.showDistance) ? 'OFF' : '');
                return;
            }

            if (al) {
                const load = al.load;
                edge.data('load', load);

                const metric = edge.data('metric');
                const parts = [];
                if (State.showMlu) parts.push(fmt(load, 2));
                if (State.showDistance) parts.push('d=' + metric);
                edge.data('label', parts.join(' '));

                if (State.colorByLoad) {
                    if (load > 1.0) edge.addClass('overloaded');
                    else if (load > 0.8) edge.addClass('high-load');
                    else if (load > 0.5) edge.addClass('medium-load');

                    // Dynamic color for all edges
                    edge.style('line-color', getLoadColor(load));
                    edge.style('target-arrow-color', getLoadColor(load));
                    edge.style('width', Math.max(1.5, Math.min(6, load * 5)));
                }
            } else {
                edge.data('label', '');
                edge.data('load', 0);
            }
        });

        // Reset node classes
        State.cy.nodes().removeClass('highlighted source-node target-node');
    }

    // ========================================================================
    // TOOLTIP
    // ========================================================================
    function showEdgeTooltip(evt) {
        const edge = evt.target;
        const arcId = edge.data('arcId');
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const al = snap.arc_loads.find(a => a.arc_id === arcId);
        if (!al) return;

        const metric = edge.data('metric');
        const tt = document.getElementById('tooltip');
        tt.classList.remove('hidden');
        tt.innerHTML = `
            <div class="tip-title">Arc ${al.arc_id}: v${al.from} → v${al.to}</div>
            <div class="tip-row"><span class="tip-label">Load:</span> <span class="tip-value" style="color:${getLoadColor(al.load)}">${fmt(al.load)}</span></div>
            <div class="tip-row"><span class="tip-label">Flow:</span> <span class="tip-value">${fmt(al.flow, 2)}</span></div>
            <div class="tip-row"><span class="tip-label">Capacity:</span> <span class="tip-value">${fmt(al.capacity, 0)}</span></div>
            <div class="tip-row"><span class="tip-label">Metric (dist):</span> <span class="tip-value">${metric}</span></div>
        `;

        const pos = evt.renderedPosition || evt.position;
        const container = document.getElementById('center-panel');
        const rect = container.getBoundingClientRect();
        tt.style.left = (pos.x + rect.left + 15) + 'px';
        tt.style.top = (pos.y + rect.top + 15) + 'px';
    }

    function showNodeHoverTooltip(evt) {
        const node = evt.target;
        const nodeId = node.data('nodeId');

        const tt = document.getElementById('tooltip');
        tt.classList.remove('hidden');

        let content = `<div class="tip-title">Node v${nodeId}</div>`;

        // Count demands from/to this node
        const snap = getCurrentSnapshot();
        if (snap) {
            const asSource = snap.demands.filter(d => d.source === nodeId).length;
            const asTarget = snap.demands.filter(d => d.target === nodeId).length;
            content += `<div class="tip-row"><span class="tip-label">Source for:</span> <span class="tip-value">${asSource} demands</span></div>`;
            content += `<div class="tip-row"><span class="tip-label">Target for:</span> <span class="tip-value">${asTarget} demands</span></div>`;
        }

        tt.innerHTML = content;

        const pos = evt.renderedPosition || evt.position;
        const container = document.getElementById('center-panel');
        const rect = container.getBoundingClientRect();
        tt.style.left = (pos.x + rect.left + 15) + 'px';
        tt.style.top = (pos.y + rect.top + 15) + 'px';
    }

    function showNodeTooltip(evt, nodeId) {
        // On click, highlight demands from/to this node
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const related = snap.demands.filter(d => d.source === nodeId || d.target === nodeId);
        if (related.length > 0) {
            // Show in demand table
            const tbody = document.getElementById('demand-tbody');
            tbody.querySelectorAll('tr').forEach(tr => {
                const did = parseInt(tr.dataset.demandId);
                const match = related.find(d => d.demand_id === did);
                if (match) {
                    tr.style.background = 'rgba(122,162,247,0.15)';
                } else {
                    tr.style.background = '';
                }
            });
        }
    }

    function hideTooltip() {
        document.getElementById('tooltip').classList.add('hidden');
    }

    // ========================================================================
    // ARC SELECTION
    // ========================================================================
    function selectArc(arcId) {
        State.selectedArcId = arcId;

        // Highlight edge in graph
        State.cy.edges().removeClass('selected-arc');
        const edge = State.cy.getElementById('e' + arcId);
        if (edge.length) edge.addClass('selected-arc');

        // Update arc detail panel
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const al = snap.arc_loads.find(a => a.arc_id === arcId);
        if (!al) return;

        const content = document.getElementById('arc-detail-content');
        content.innerHTML = `
            <div style="margin-bottom:10px;">
                <strong style="color:var(--accent-blue)">Arc ${arcId}: v${al.from} → v${al.to}</strong>
            </div>
            <div class="tip-row"><span class="tip-label">Load:</span> <span class="tip-value" style="color:${getLoadColor(al.load)}">${fmt(al.load)}</span></div>
            <div class="tip-row"><span class="tip-label">Flow:</span> <span class="tip-value">${fmt(al.flow, 2)}</span></div>
            <div class="tip-row"><span class="tip-label">Capacity:</span> <span class="tip-value">${fmt(al.capacity, 0)}</span></div>
            <div class="tip-row"><span class="tip-label">Metric:</span> <span class="tip-value">${edge.length ? edge.data('metric') : '—'}</span></div>
        `;

        // Compute actual ECMP flow contribution of each demand on this arc
        const contributions = getDemandsOnArc(arcId);
        // Store for use by bulk actions
        State._arcContributions = contributions;
        const listEl = document.getElementById('arc-demands-list');

        if (contributions.length > 0) {
            // Sort by contribution descending (highest contributor first)
            contributions.sort((a, b) => b.contribution - a.contribution);
            const totalContrib = contributions.reduce((s, c) => s + c.contribution, 0);

            // Bulk action toolbar
            let html = `<div class="arc-demands-toolbar">
                <div class="arc-demands-summary">
                    ${contributions.length} demand${contributions.length > 1 ? 's' : ''}
                    · flow: ${fmt(totalContrib, 2)} / ${fmt(al.capacity, 0)}
                </div>
                <div class="arc-demands-actions">
                    <button class="btn-sm" onclick="window.TASRApp.arcSelectAllDemands()" title="Select all contributing demands">☑ Select All</button>
                    <button class="btn-sm" onclick="window.TASRApp.arcShowAllRoutes()" title="Show routes of all contributing demands">⤳ Show All Routes</button>
                </div>
            </div>`;

            contributions.forEach((c, idx) => {
                const pct = al.flow > 0 ? (c.contribution / al.flow * 100) : 0;
                const dId = c.demand.demand_id;
                const isSelected = State.selectedDemandIds.has(dId);
                html += `
                    <div class="arc-demand-contrib${isSelected ? ' selected' : ''}" data-demand-id="${dId}">
                        <div class="arc-demand-contrib-header">
                            <span class="arc-demand-rank">#${idx + 1}</span>
                            <span class="label" onclick="window.TASRApp.selectDemand(${dId})" style="cursor:pointer"
                                title="Open demand detail">D${dId} (v${c.demand.source}→v${c.demand.target})</span>
                            <span class="value">${fmt(c.contribution, 2)}</span>
                        </div>
                        <div class="arc-demand-contrib-bar">
                            <div class="arc-demand-contrib-bar-fill" style="width:${Math.min(pct, 100).toFixed(1)}%"></div>
                        </div>
                        <div class="arc-demand-contrib-detail">
                            vol=${fmt(c.demand.volume, 1)} · ${fmt(pct, 1)}% of flow
                            · wp=[${c.demand.waypoints.join(',')}]
                        </div>
                        <div class="arc-demand-contrib-actions">
                            <button class="btn-xs" onclick="window.TASRApp.arcToggleDemand(${dId})" title="Toggle selection">
                                ${isSelected ? '☑' : '☐'} Select
                            </button>
                            <button class="btn-xs" onclick="window.TASRApp.arcShowDemandFlow(${dId})" title="Show ECMP flow split">
                                ⤳ Flow
                            </button>
                            <button class="btn-xs" onclick="window.TASRApp.arcEditDemandWaypoints(${dId})" title="Edit waypoints">
                                ✎ Waypoints
                            </button>
                        </div>
                    </div>`;
            });
            listEl.innerHTML = html;
        } else {
            listEl.innerHTML = '<div class="empty-state">No demands routing through this arc</div>';
        }

        // Switch to arc detail tab
        switchTab('arc-detail-tab');
    }

    /** Toggle a demand's selection from the arc contribution list */
    function arcToggleDemand(demandId) {
        if (State.selectedDemandIds.has(demandId)) {
            State.selectedDemandIds.delete(demandId);
        } else {
            State.selectedDemandIds.add(demandId);
        }
        // Update the checkbox appearance in the arc list without re-running expensive ECMP
        const listEl = document.getElementById('arc-demands-list');
        if (listEl) {
            const item = listEl.querySelector(`.arc-demand-contrib[data-demand-id="${demandId}"]`);
            if (item) {
                const isNowSelected = State.selectedDemandIds.has(demandId);
                item.classList.toggle('selected', isNowSelected);
                const btn = item.querySelector('.arc-demand-contrib-actions .btn-xs');
                if (btn) btn.innerHTML = `${isNowSelected ? '☑' : '☐'} Select`;
            }
        }
        updateDemandTable();
    }

    /** Select all demands contributing to the currently viewed arc */
    function arcSelectAllDemands() {
        if (!State._arcContributions) return;
        State._arcContributions.forEach(c => State.selectedDemandIds.add(c.demand.demand_id));
        // Update checkboxes in-place
        const listEl = document.getElementById('arc-demands-list');
        if (listEl) {
            listEl.querySelectorAll('.arc-demand-contrib').forEach(item => {
                item.classList.add('selected');
                const btn = item.querySelector('.arc-demand-contrib-actions .btn-xs');
                if (btn) btn.innerHTML = '☑ Select';
            });
        }
        updateDemandTable();
    }

    /** Show multi-route overlay for all demands contributing to the current arc */
    function arcShowAllRoutes() {
        if (!State._arcContributions || State._arcContributions.length === 0) return;
        State.selectedDemandIds.clear();
        State._arcContributions.forEach(c => State.selectedDemandIds.add(c.demand.demand_id));
        if (!State.graph) buildGraph();
        highlightMultipleDemandRoutes();
        updateDemandTable();
    }

    /** Show ECMP flow overlay for a specific demand (from arc view) */
    function arcShowDemandFlow(demandId) {
        const snap = getCurrentSnapshot();
        if (!snap) return;
        const demand = snap.demands.find(d => d.demand_id === demandId);
        if (!demand) return;
        if (!State.graph) buildGraph();
        showDemandFlowOverlay(demand);
    }

    /** Open waypoint editor for a specific demand (from arc view) */
    function arcEditDemandWaypoints(demandId) {
        selectDemand(demandId);
        // The selectDemand call already opens the waypoint editor via showWaypointEditor
    }

    /**
     * Compute the actual ECMP flow contribution of each demand on a given arc.
     * Returns array of { demand, contribution } sorted by contribution descending.
     * Uses computeEcmpFlow per segment for accurate flow-splitting calculation.
     */
    function getDemandsOnArc(arcId) {
        const snap = getCurrentSnapshot();
        if (!snap) return [];
        if (!State.graph) buildGraph();

        const result = [];

        for (const d of snap.demands) {
            if (d.volume <= 0) continue;
            const segEndpoints = [d.source, ...d.waypoints, d.target];
            let totalContrib = 0;
            const visitedNodes = State.noRevisit ? new Set([d.source]) : null;

            for (let i = 0; i < segEndpoints.length - 1; i++) {
                const arcFlows = computeEcmpFlow(segEndpoints[i], segEndpoints[i + 1], d.volume, visitedNodes);
                const flow = arcFlows.get(arcId);
                if (flow && flow > 1e-9) {
                    totalContrib += flow;
                }
                if (visitedNodes) {
                    arcFlows.forEach((_, aid) => {
                        const arc = State.data.arcs.find(a => a.id === aid);
                        if (arc) { visitedNodes.add(arc.from); visitedNodes.add(arc.to); }
                    });
                }
            }

            if (totalContrib > 1e-9) {
                result.push({ demand: d, contribution: totalContrib });
            }
        }
        return result;
    }

    // ========================================================================
    // DEMAND SELECTION
    // ========================================================================
    function selectDemand(demandId) {
        State.selectedDemandId = demandId;
        State.showingMultiRoutes = false;

        const snap = getCurrentSnapshot();
        if (!snap) return;

        const demand = snap.demands.find(d => d.demand_id === demandId);
        if (!demand) return;

        // Ensure graph is built for route computation
        if (!State.graph) buildGraph();

        // Highlight in table
        const tbody = document.getElementById('demand-tbody');
        tbody.querySelectorAll('tr').forEach(tr => {
            tr.classList.toggle('selected', parseInt(tr.dataset.demandId) === demandId);
        });

        // Show detail
        const detail = document.getElementById('demand-detail-content');
        const segColors = ['#7aa2f7', '#9ece6a', '#ff9e64', '#e0af68', '#bb9af7', '#f7768e'];
        const segmentEndpoints = [demand.source, ...demand.waypoints, demand.target];
        let segmentHtml = '';
        if (State.graph) {
            const visitedNodesInfo = State.noRevisit ? new Set([demand.source]) : null;
            for (let i = 0; i < segmentEndpoints.length - 1; i++) {
                const from = segmentEndpoints[i];
                const to = segmentEndpoints[i + 1];
                const color = segColors[i % segColors.length];
                const result = dijkstra(from, to);
                const distStr = result ? fmt(result.dist) : '∞';
                const ecmpResult = dijkstraAllShortestPaths(from, to, visitedNodesInfo);
                const numPaths = ecmpResult ? ecmpResult.shortestArcs.size : 0;
                if (visitedNodesInfo && ecmpResult) {
                    ecmpResult.shortestArcs.forEach(aid => {
                        const arc = State.data.arcs.find(a => a.id === aid);
                        if (arc) { visitedNodesInfo.add(arc.from); visitedNodesInfo.add(arc.to); }
                    });
                }
                segmentHtml += `<div class="seg-row">
                    <span class="segment-indicator" style="background:${color}"></span>
                    <span class="seg-label">Seg ${i + 1}:</span>
                    <span class="seg-path">v${from} → v${to}</span>
                    <span class="seg-dist">d=${distStr} (${numPaths} arcs)</span>
                </div>`;
            }
        }

        detail.innerHTML = `
            <div><strong>Demand ${demand.demand_id}</strong></div>
            <div>Source: v${demand.source} → Target: v${demand.target}</div>
            <div>Volume: ${fmt(demand.volume, 2)}</div>
            <div>Waypoints: [${demand.waypoints.join(', ')}]</div>
            <div>Segments: ${demand.waypoints.length + 1}</div>
            <div>Congestion contributor: ${demand.is_candidate ? '<span class="text-red">Yes</span>' : '<span class="text-green">No</span>'}</div>
            ${demand.is_candidate ? '<div>Contribution: ' + fmt(demand.contribution, 2) + '</div>' : ''}
            ${segmentHtml ? '<div class="segment-info">' + segmentHtml + '</div>' : ''}
        `;

        highlightDemandRoute(demand);

        // Show waypoint editor
        showWaypointEditor(demand);
    }

    function highlightDemandRoute(demand) {
        if (!State.cy) return;
        if (!State.graph) buildGraph();

        // Exit flow overlay if active
        if (State.showingFlow) exitFlowOverlay();

        // Clear all dynamic inline styles from multi-route/flow overlays
        State.cy.edges().forEach(e => { e.removeStyle(); });
        State.cy.nodes().forEach(n => { n.removeStyle(); });

        // Reset previous highlights
        State.cy.nodes().removeClass('highlighted source-node target-node waypoint-node flow-dimmed flow-node');
        State.cy.edges().removeClass('highlighted-route ecmp-path segment-0 segment-1 segment-2 segment-3 segment-4 segment-5 flow-dimmed flow-edge selected-arc');

        // Restore load-based styling first
        State.cy.style().fromJson(getCytoscapeStyle()).update();
        updateGraphLoads();

        // Highlight source and target nodes
        const srcNode = State.cy.getElementById('n' + demand.source);
        const tgtNode = State.cy.getElementById('n' + demand.target);
        if (srcNode.length) srcNode.addClass('source-node');
        if (tgtNode.length) tgtNode.addClass('target-node');

        // Highlight waypoint nodes
        demand.waypoints.forEach(wp => {
            const wpNode = State.cy.getElementById('n' + wp);
            if (wpNode.length) wpNode.addClass('waypoint-node');
        });

        // Build segment path and highlight using ALL shortest paths (ECMP)
        if (State.graph) {
            const segmentEndpoints = [demand.source, ...demand.waypoints, demand.target];
            const segColors = ['segment-0', 'segment-1', 'segment-2', 'segment-3', 'segment-4', 'segment-5'];
            const visitedNodesRoute = State.noRevisit ? new Set([demand.source]) : null;

            for (let i = 0; i < segmentEndpoints.length - 1; i++) {
                const from = segmentEndpoints[i];
                const to = segmentEndpoints[i + 1];
                const segClass = segColors[i % segColors.length];

                const ecmpResult = dijkstraAllShortestPaths(from, to, visitedNodesRoute);
                if (ecmpResult) {
                    const primarySet = new Set(ecmpResult.primaryArcs);

                    // Highlight all ECMP arcs
                    ecmpResult.shortestArcs.forEach(arcId => {
                        const edge = State.cy.getElementById('e' + arcId);
                        if (edge.length) {
                            if (primarySet.has(arcId)) {
                                // Primary shortest path — solid color per segment
                                edge.addClass(segClass);
                            } else {
                                // Alternative ECMP path — dashed
                                edge.addClass('ecmp-path');
                            }
                        }
                    });

                    // Track visited nodes for no-revisit
                    if (visitedNodesRoute) {
                        ecmpResult.shortestArcs.forEach(aid => {
                            const arc = State.data.arcs.find(a => a.id === aid);
                            if (arc) { visitedNodesRoute.add(arc.from); visitedNodesRoute.add(arc.to); }
                        });
                    }
                }
            }
        } else {
            // Fallback: highlight direct edges between consecutive path nodes
            const pathNodes = [demand.source, ...demand.waypoints, demand.target];
            for (let i = 0; i < pathNodes.length - 1; i++) {
                const from = pathNodes[i];
                const to = pathNodes[i + 1];
                State.cy.edges().forEach(edge => {
                    const src = edge.data('source');
                    const tgt = edge.data('target');
                    if (src === 'n' + from && tgt === 'n' + to) {
                        edge.addClass('highlighted-route');
                    }
                });
            }
        }
    }

    function clearSelection() {
        State.selectedDemandId = null;
        State.selectedArcId = null;
        State.editingWaypoints = null;
        State.originalWaypoints = null;
        State.showingMultiRoutes = false;

        if (State.showingFlow) exitFlowOverlay();

        hideWaypointEditor();

        if (State.cy) {
            State.cy.nodes().removeClass('highlighted source-node target-node waypoint-node flow-dimmed flow-node');
            State.cy.edges().removeClass('highlighted-route selected-arc ecmp-path segment-0 segment-1 segment-2 segment-3 segment-4 segment-5 flow-dimmed flow-edge');
            // Remove dynamic multi-route styles
            State.cy.edges().forEach(e => { e.removeStyle(); });
            State.cy.nodes().forEach(n => { n.removeStyle(); });
        }

        document.getElementById('demand-detail-content').textContent = 'Select a demand to view details';
        const tbody = document.getElementById('demand-tbody');
        tbody.querySelectorAll('tr').forEach(tr => {
            tr.classList.remove('selected');
            tr.style.background = '';
        });

        // Restore normal load-based styling
        updateGraphLoads();
    }

    // ========================================================================
    // MULTI-DEMAND ROUTE VISUALIZATION
    // ========================================================================

    function highlightMultipleDemandRoutes() {
        const snap = getCurrentSnapshot();
        if (!snap || !State.cy || !State.graph) return;

        const demandIds = Array.from(State.selectedDemandIds);
        if (demandIds.length === 0) return;

        State.showingMultiRoutes = true;
        if (State.showingFlow) exitFlowOverlay();

        // Clear previous
        State.cy.nodes().removeClass('highlighted source-node target-node waypoint-node flow-dimmed flow-node');
        State.cy.edges().removeClass('highlighted-route selected-arc ecmp-path segment-0 segment-1 segment-2 segment-3 segment-4 segment-5 flow-dimmed flow-edge');
        State.cy.edges().forEach(e => { e.removeStyle(); });
        State.cy.nodes().forEach(n => { n.removeStyle(); });

        // Dim everything first
        State.cy.edges().addClass('flow-dimmed');
        State.cy.nodes().addClass('flow-dimmed');

        // Track which arcs/nodes are used (to un-dim)
        const usedArcs = new Map(); // arcId -> { color, demandIds }
        const usedNodes = new Set();

        demandIds.forEach((dId, colorIdx) => {
            const demand = snap.demands.find(d => d.demand_id === dId);
            if (!demand) return;

            const color = getRouteColor(colorIdx);
            const segmentEndpoints = [demand.source, ...demand.waypoints, demand.target];

            // Mark source/target/waypoint nodes
            usedNodes.add(demand.source);
            usedNodes.add(demand.target);
            demand.waypoints.forEach(w => usedNodes.add(w));

            // Find ECMP paths for each segment
            const visitedNodesMulti = State.noRevisit ? new Set([demand.source]) : null;
            for (let i = 0; i < segmentEndpoints.length - 1; i++) {
                const from = segmentEndpoints[i];
                const to = segmentEndpoints[i + 1];

                const ecmpResult = dijkstraAllShortestPaths(from, to, visitedNodesMulti);
                if (ecmpResult) {
                    ecmpResult.shortestArcs.forEach(arcId => {
                        if (!usedArcs.has(arcId)) {
                            usedArcs.set(arcId, { color: color, demandIds: [dId] });
                        } else {
                            const entry = usedArcs.get(arcId);
                            entry.demandIds.push(dId);
                            // Keep the first demand's color (or blend — use first)
                        }

                        // Un-dim the arc endpoints
                        const arc = State.data.arcs.find(a => a.id === arcId);
                        if (arc) {
                            usedNodes.add(arc.from);
                            usedNodes.add(arc.to);
                        }
                    });

                    // Track visited nodes for no-revisit
                    if (visitedNodesMulti) {
                        ecmpResult.shortestArcs.forEach(aid => {
                            const arc = State.data.arcs.find(a => a.id === aid);
                            if (arc) { visitedNodesMulti.add(arc.from); visitedNodesMulti.add(arc.to); }
                        });
                    }
                }
            }
        });

        // Apply colors to arcs
        usedArcs.forEach(({ color, demandIds: dIds }, arcId) => {
            const edge = State.cy.getElementById('e' + arcId);
            if (!edge.length) return;
            edge.removeClass('flow-dimmed');

            // If shared by multiple demands, use white dashed
            if (dIds.length > 1) {
                edge.style({
                    'line-color': '#ffffff',
                    'target-arrow-color': '#ffffff',
                    'width': 4,
                    'z-index': 999,
                    'line-style': 'dashed',
                    'line-dash-pattern': [6, 3],
                });
            } else {
                edge.style({
                    'line-color': color,
                    'target-arrow-color': color,
                    'width': 4,
                    'z-index': 999,
                });
            }
        });

        // Un-dim and style nodes
        usedNodes.forEach(nodeId => {
            const node = State.cy.getElementById('n' + nodeId);
            if (node.length) {
                node.removeClass('flow-dimmed');
                node.addClass('flow-node');
            }
        });

        // Style source/target nodes per demand
        demandIds.forEach((dId, colorIdx) => {
            const demand = snap.demands.find(d => d.demand_id === dId);
            if (!demand) return;
            const color = getRouteColor(colorIdx);

            const srcNode = State.cy.getElementById('n' + demand.source);
            if (srcNode.length) {
                srcNode.style('border-color', color);
                srcNode.style('border-width', 3);
            }
            const tgtNode = State.cy.getElementById('n' + demand.target);
            if (tgtNode.length) {
                tgtNode.style('border-color', color);
                tgtNode.style('border-width', 3);
            }
        });

        // Build legend in demand detail panel
        const detail = document.getElementById('demand-detail-content');
        let html = '<div class="multi-route-legend">';
        html += '<div class="multi-route-legend-title">Selected Demand Routes (' + demandIds.length + ')</div>';
        demandIds.forEach((dId, colorIdx) => {
            const demand = snap.demands.find(d => d.demand_id === dId);
            if (!demand) return;
            const color = getRouteColor(colorIdx);
            html += `<div class="multi-route-legend-item" onclick="window.TASRApp.selectDemand(${dId})">
                <span class="legend-swatch" style="background:${color}"></span>
                <span>D${dId}: v${demand.source}→v${demand.target} (vol=${fmt(demand.volume, 1)}, wp=[${demand.waypoints.join(',')}])</span>
            </div>`;
        });

        // Shared arcs note
        const sharedCount = Array.from(usedArcs.values()).filter(v => v.demandIds.length > 1).length;
        if (sharedCount > 0) {
            html += `<div style="margin-top:6px;font-size:10px;color:var(--text-muted)">
                White dashed = shared by multiple demands (${sharedCount} arcs)
            </div>`;
        }

        html += '<div style="margin-top:8px"><button class="btn btn-sm btn-secondary" id="btn-exit-multi-routes">Exit Multi-Route View</button></div>';
        html += '</div>';
        detail.innerHTML = html;

        document.getElementById('btn-exit-multi-routes').addEventListener('click', function () {
            State.showingMultiRoutes = false;
            clearSelection();
        });
    }

    // ========================================================================
    // UPDATE FUNCTIONS
    // ========================================================================
    function updateAll() {
        updateNavigation();
        updateMetrics();
        updateTopLoads();
        updateCongestedArcsList();
        updateViolations();
        updateGraphLoads();
        updateDemandTable();
        updateCharts();
        updateMoveLog();
    }

    function updateNavigation() {
        const times = getAllTimeSlots();
        const snapsForTime = getSnapshotsForTime(State.currentTimeSlot);

        document.getElementById('time-display').textContent =
            `${State.currentTimeSlot} / ${(State.data ? State.data.num_time_slots - 1 : 0)}`;
        document.getElementById('iter-display').textContent =
            `${State.currentSnapshotIdx} / ${Math.max(0, snapsForTime.length - 1)}`;

        const slider = document.getElementById('iter-slider');
        slider.max = Math.max(0, snapsForTime.length - 1);
        slider.value = State.currentSnapshotIdx;
    }

    function updateMetrics() {
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const mluEl = document.getElementById('mlu-value');
        mluEl.textContent = fmt(snap.objective.mlu);
        mluEl.className = 'metric-value ' + getLoadClass(snap.objective.mlu);

        document.getElementById('budget-value').textContent =
            `${snap.budget.used} / ${snap.budget.allowed}`;
        document.getElementById('violations-value').textContent =
            snap.violations.length;
        document.getElementById('demands-count').textContent =
            snap.demands.length;

        const violEl = document.getElementById('violations-value');
        violEl.className = 'metric-value ' + (snap.violations.length > 0 ? 'danger' : 'good');
    }

    function updateTopLoads() {
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const list = document.getElementById('top-loads-list');
        list.innerHTML = '';

        // Sort all arc loads descending to get top loaded arcs with names
        const sortedArcLoads = [...snap.arc_loads].sort((a, b) => b.load - a.load);
        const topArcs = sortedArcLoads.slice(0, 10);

        topArcs.forEach((al, i) => {
            const item = document.createElement('div');
            item.className = 'list-item';
            item.style.cursor = 'pointer';
            item.innerHTML = `
                <span class="label">#${i + 1}</span>
                <span class="top-load-arc-id">a${al.arc_id}</span>
                <span class="top-load-arc-name">v${al.from}→v${al.to}</span>
                <span class="top-load-time">t=${snap.time}</span>
                <span class="value" style="color:${getLoadColor(al.load)}">${fmt(al.load)}</span>
                <div class="load-bar"><div class="load-bar-fill" style="width:${Math.min(100, al.load * 100)}%;background:${getLoadColor(al.load)}"></div></div>
            `;
            item.addEventListener('click', () => selectArc(al.arc_id));
            list.appendChild(item);
        });
    }

    function updateCongestedArcsList() {
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const label = document.getElementById('congested-time-label');
        label.textContent = '(t=' + snap.time + ')';

        const list = document.getElementById('congested-arcs-list');
        list.innerHTML = '';

        // Filter out zero-load and intervention arcs
        const interventionSet = new Set(snap.intervention_arcs || []);
        const filtered = [...snap.arc_loads]
            .filter(a => a.load > 0 && !interventionSet.has(a.arc_id));

        // Sort by selected key
        const key = State.arcSortKey;
        const asc = State.arcSortAsc;
        filtered.sort((a, b) => {
            let va, vb;
            if (key === 'id')   { va = a.arc_id; vb = b.arc_id; }
            else if (key === 'from') { va = a.from; vb = b.from; }
            else if (key === 'to')   { va = a.to;   vb = b.to; }
            else                     { va = a.load;  vb = b.load; }
            return asc ? (va < vb ? -1 : va > vb ? 1 : 0) : (vb < va ? -1 : vb > va ? 1 : 0);
        });

        if (filtered.length === 0) {
            list.innerHTML = '<div class="empty-state">No loaded arcs</div>';
            return;
        }

        filtered.forEach((al, i) => {
            const item = document.createElement('div');
            item.className = 'congested-arc-item' + (al.arc_id === State.selectedArcId ? ' selected' : '');
            item.innerHTML = `
                <span class="congested-arc-rank">#${i + 1}</span>
                <span class="congested-arc-id">a${al.arc_id}</span>
                <span class="congested-arc-name">v${al.from}→v${al.to}</span>
                <span class="congested-arc-load" style="color:${getLoadColor(al.load)}">${fmt(al.load, 3)}</span>
                <span class="congested-arc-bar"><span class="congested-arc-bar-fill" style="width:${Math.min(100, al.load * 100)}%;background:${getLoadColor(al.load)}"></span></span>
            `;
            item.addEventListener('click', () => selectArc(al.arc_id));
            list.appendChild(item);
        });
    }

    function updateViolations() {
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const list = document.getElementById('violations-list');
        list.innerHTML = '';

        if (snap.violations.length === 0) {
            list.innerHTML = '<div class="empty-state">No violations</div>';
            return;
        }

        snap.violations.forEach(v => {
            const item = document.createElement('div');
            item.className = 'violation-item';
            item.textContent = v;
            list.appendChild(item);
        });
    }

    function updateDemandTable() {
        const snap = getCurrentSnapshot();
        if (!snap) return;

        const tbody = document.getElementById('demand-tbody');
        tbody.innerHTML = '';

        const searchTerm = document.getElementById('demand-search').value.toLowerCase();

        // Build color index for selected demands
        const selectedArr = Array.from(State.selectedDemandIds);

        // Filter demands
        let demands = snap.demands.filter(d => {
            if (!searchTerm) return true;
            const searchStr = `${d.demand_id} ${d.source} ${d.target} ${d.volume}`.toLowerCase();
            return searchStr.includes(searchTerm);
        });

        // Sort demands
        const key = State.demandSortKey;
        const asc = State.demandSortAsc;
        demands = demands.slice().sort((a, b) => {
            let va, vb;
            switch (key) {
                case 'source': va = a.source; vb = b.source; break;
                case 'target': va = a.target; vb = b.target; break;
                case 'volume': va = a.volume; vb = b.volume; break;
                default: va = a.demand_id; vb = b.demand_id; break;
            }
            return asc ? (va - vb) : (vb - va);
        });

        // Update header sort arrows
        document.querySelectorAll('#demand-table .sortable-th').forEach(th => {
            const arrow = th.querySelector('.sort-arrow');
            arrow.className = 'sort-arrow';
            if (th.dataset.sort === key) {
                arrow.classList.add(asc ? 'asc' : 'desc');
            }
        });

        demands.forEach(d => {
            const tr = document.createElement('tr');
            tr.dataset.demandId = d.demand_id;
            if (d.is_candidate) tr.classList.add('congested');
            if (d.demand_id === State.selectedDemandId) tr.classList.add('selected');

            const isChecked = State.selectedDemandIds.has(d.demand_id);
            const colorIdx = selectedArr.indexOf(d.demand_id);
            const colorSwatch = isChecked ? `<span class="route-color-swatch" style="background:${getRouteColor(colorIdx)}"></span>` : '';

            tr.innerHTML = `
                <td class="col-chk"><input type="checkbox" class="demand-chk" data-id="${d.demand_id}" ${isChecked ? 'checked' : ''}></td>
                <td>${colorSwatch}${d.demand_id}</td>
                <td>v${d.source}</td>
                <td>v${d.target}</td>
                <td>${fmt(d.volume, 1)}</td>
                <td>${d.waypoints.length > 0 ? d.waypoints.join(',') : '—'}</td>
                <td>${d.is_candidate ? '<span class="text-red">●</span>' : ''}</td>
            `;

            // Click row = single-select for detail view
            tr.addEventListener('click', (e) => {
                if (e.target.type === 'checkbox') return;
                selectDemand(d.demand_id);
            });

            // Checkbox change = multi-select
            const chk = tr.querySelector('.demand-chk');
            chk.addEventListener('change', function (e) {
                e.stopPropagation();
                if (this.checked) {
                    State.selectedDemandIds.add(d.demand_id);
                } else {
                    State.selectedDemandIds.delete(d.demand_id);
                }
                updateMultiSelectCount();
                updateDemandTable();
            });

            tbody.appendChild(tr);
        });

        updateMultiSelectCount();
    }

    function updateMultiSelectCount() {
        const countEl = document.getElementById('multi-select-count');
        const n = State.selectedDemandIds.size;
        countEl.textContent = n > 0 ? n + ' selected' : '';
        // Sync header checkbox
        const headerChk = document.getElementById('chk-select-all');
        const snap = getCurrentSnapshot();
        if (snap && headerChk) {
            headerChk.checked = n > 0 && n === snap.demands.length;
            headerChk.indeterminate = n > 0 && n < snap.demands.length;
        }
    }

    // ========================================================================
    // CHARTS
    // ========================================================================
    function updateCharts() {
        if (!State.data) return;

        const plotConfig = { displayModeBar: false, responsive: true };
        const plotLayout = {
            paper_bgcolor: '#24283b',
            plot_bgcolor: '#1a1b26',
            font: { color: '#c0caf5', size: 10 },
            margin: { t: 30, b: 30, l: 40, r: 10 },
            xaxis: { gridcolor: '#3b4261', color: '#7982a9' },
            yaxis: { gridcolor: '#3b4261', color: '#7982a9' },
        };

        // MLU Chart - across time slots
        const mluData = [];
        const timeLabels = [];
        State.data.snapshots.forEach(snap => {
            timeLabels.push('t=' + snap.time);
            mluData.push(snap.objective.mlu);
        });

        Plotly.newPlot('mlu-chart', [{
            x: timeLabels,
            y: mluData,
            type: 'scatter',
            mode: 'lines+markers',
            line: { color: '#7aa2f7', width: 2 },
            marker: { color: '#7aa2f7', size: 6 },
            name: 'MLU'
        }], {
            ...plotLayout,
            title: { text: 'MLU per Snapshot', font: { size: 12, color: '#c0caf5' } },
            yaxis: { ...plotLayout.yaxis, title: 'MLU' },
        }, plotConfig);

        // Load Distribution Chart
        const snap = getCurrentSnapshot();
        if (snap) {
            const loads = snap.arc_loads.map(a => a.load).sort((a, b) => b - a);
            const colors = loads.map(l => getLoadColor(l));

            Plotly.newPlot('load-distribution-chart', [{
                y: loads,
                type: 'bar',
                marker: { color: colors },
                name: 'Arc Loads'
            }], {
                ...plotLayout,
                title: { text: 'Arc Load Distribution (sorted)', font: { size: 12, color: '#c0caf5' } },
                yaxis: { ...plotLayout.yaxis, title: 'Utilization' },
                xaxis: { ...plotLayout.xaxis, title: 'Arc rank', showticklabels: false },
            }, plotConfig);

            // Budget Chart
            const budgetUsed = State.data.snapshots.map(s => s.budget.used);
            const budgetAllowed = State.data.snapshots.map(s => s.budget.allowed);
            const snapLabels = State.data.snapshots.map((s, i) => 't=' + s.time);

            Plotly.newPlot('budget-chart', [
                {
                    x: snapLabels, y: budgetUsed,
                    type: 'bar', name: 'Used',
                    marker: { color: '#ff9e64' }
                },
                {
                    x: snapLabels, y: budgetAllowed,
                    type: 'scatter', mode: 'lines',
                    name: 'Allowed',
                    line: { color: '#f7768e', dash: 'dash', width: 2 }
                }
            ], {
                ...plotLayout,
                title: { text: 'Budget Usage', font: { size: 12, color: '#c0caf5' } },
                yaxis: { ...plotLayout.yaxis, title: 'Budget' },
                barmode: 'overlay',
                showlegend: true,
                legend: { font: { size: 9 } },
            }, plotConfig);
        }
    }

    function updateMoveLog() {
        const list = document.getElementById('move-log-list');
        list.innerHTML = '';

        if (!State.data) return;

        State.data.snapshots.forEach((snap, i) => {
            if (snap.move && snap.move.type !== 'none') {
                const item = document.createElement('div');
                item.className = 'move-item';
                item.innerHTML = `
                    <div><span class="move-type">[${snap.move.type}]</span> t=${snap.time} iter=${snap.iteration}</div>
                    <div class="move-detail">${snap.move.details || ''} ${snap.move.demand_id >= 0 ? '(demand ' + snap.move.demand_id + ')' : ''}</div>
                `;
                list.appendChild(item);
            }
        });

        if (list.children.length === 0) {
            list.innerHTML = '<div class="empty-state">No moves recorded</div>';
        }
    }

    // ========================================================================
    // NAVIGATION
    // ========================================================================
    function nextTimeSlot() {
        if (!State.data) return;
        if (State.currentTimeSlot < State.data.num_time_slots - 1) {
            State.currentTimeSlot++;
            State.currentSnapshotIdx = 0;
            updateAll();
        }
    }

    function prevTimeSlot() {
        if (!State.data) return;
        if (State.currentTimeSlot > 0) {
            State.currentTimeSlot--;
            State.currentSnapshotIdx = 0;
            updateAll();
        }
    }

    function nextIteration() {
        const snaps = getSnapshotsForTime(State.currentTimeSlot);
        if (State.currentSnapshotIdx < snaps.length - 1) {
            State.currentSnapshotIdx++;
            updateAll();
        }
    }

    function prevIteration() {
        if (State.currentSnapshotIdx > 0) {
            State.currentSnapshotIdx--;
            updateAll();
        }
    }

    // ========================================================================
    // TAB MANAGEMENT
    // ========================================================================
    function switchTab(tabId) {
        document.querySelectorAll('#right-panel .tab').forEach(t => t.classList.remove('active'));
        document.querySelectorAll('#right-panel .tab-content').forEach(t => t.classList.remove('active'));

        const tab = document.querySelector(`[data-tab="${tabId}"]`);
        if (tab) tab.classList.add('active');
        const content = document.getElementById(tabId);
        if (content) content.classList.add('active');
    }

    // ========================================================================
    // EVENT BINDING
    // ========================================================================
    function bindEvents() {
        // File input
        document.getElementById('file-input').addEventListener('change', function (e) {
            const file = e.target.files[0];
            if (!file) return;
            const reader = new FileReader();
            reader.onload = function (ev) {
                try {
                    const data = JSON.parse(ev.target.result);
                    loadData(data);
                } catch (err) {
                    alert('Error parsing JSON: ' + err.message);
                }
            };
            reader.readAsText(file);
        });

        // Load sample
        document.getElementById('btn-load-sample').addEventListener('click', function () {
            // Try to load a default file
            fetch('data/solver_state_01.json')
                .then(r => { if (!r.ok) throw new Error('Not found'); return r.json(); })
                .then(data => loadData(data))
                .catch(() => {
                    // Generate demo data
                    loadData(generateDemoData());
                });
        });

        // Navigation
        document.getElementById('btn-next-time').addEventListener('click', nextTimeSlot);
        document.getElementById('btn-prev-time').addEventListener('click', prevTimeSlot);
        document.getElementById('btn-next-iter').addEventListener('click', nextIteration);
        document.getElementById('btn-prev-iter').addEventListener('click', prevIteration);

        document.getElementById('iter-slider').addEventListener('input', function () {
            State.currentSnapshotIdx = parseInt(this.value);
            updateAll();
        });

        // Graph controls
        document.getElementById('btn-fit').addEventListener('click', () => State.cy && State.cy.fit());
        document.getElementById('btn-zoom-in').addEventListener('click', () => State.cy && State.cy.zoom(State.cy.zoom() * 1.3));
        document.getElementById('btn-zoom-out').addEventListener('click', () => State.cy && State.cy.zoom(State.cy.zoom() / 1.3));

        document.getElementById('chk-show-labels').addEventListener('change', function () {
            State.showLabels = this.checked;
            if (State.cy) State.cy.style().fromJson(getCytoscapeStyle()).update();
        });

        document.getElementById('chk-show-mlu').addEventListener('change', function () {
            State.showMlu = this.checked;
            if (State.cy) State.cy.style().fromJson(getCytoscapeStyle()).update();
            updateGraphLoads();
        });

        document.getElementById('chk-show-distance').addEventListener('change', function () {
            State.showDistance = this.checked;
            if (State.cy) State.cy.style().fromJson(getCytoscapeStyle()).update();
            updateGraphLoads();
        });

        document.getElementById('chk-color-by-load').addEventListener('change', function () {
            State.colorByLoad = this.checked;
            updateGraphLoads();
        });

        document.getElementById('chk-no-revisit').addEventListener('change', function () {
            State.noRevisit = this.checked;
            // Re-render current selection to reflect the change
            if (State.showingFlow && State.flowDemandId !== null) {
                const snap = getCurrentSnapshot();
                const demand = snap ? snap.demands.find(d => d.demand_id === State.flowDemandId) : null;
                if (demand) showDemandFlowOverlay(demand);
            } else if (State.showingMultiRoutes) {
                showMultiDemandRoutes(Array.from(State.selectedDemandIds));
            } else if (State.selectedDemandId !== null) {
                selectDemand(State.selectedDemandId);
            }
        });

        document.getElementById('layout-select').addEventListener('change', function () {
            State.currentLayout = this.value;
        });

        document.getElementById('btn-relayout').addEventListener('click', function () {
            if (!State.cy) return;
            const layout = State.cy.layout(getLayoutConfig(State.currentLayout));
            layout.run();
        });

        // Action buttons
        document.getElementById('btn-highlight-interventions').addEventListener('click', function () {
            if (!State.cy) return;
            const snap = getCurrentSnapshot();
            if (!snap) return;
            State.cy.edges().removeClass('intervention');
            (snap.intervention_arcs || []).forEach(arcId => {
                const edge = State.cy.getElementById('e' + arcId);
                if (edge.length) edge.addClass('intervention');
            });
        });

        document.getElementById('btn-reset-view').addEventListener('click', function () {
            clearSelection();
            if (State.cy) { State.cy.fit(); }
            updateGraphLoads();
        });

        document.getElementById('btn-export-state').addEventListener('click', function () {
            const snap = getCurrentSnapshot();
            if (!snap) return;
            const blob = new Blob([JSON.stringify(snap, null, 2)], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `snapshot_t${snap.time}_iter${snap.iteration}.json`;
            a.click();
            URL.revokeObjectURL(url);
        });

        // Demand search
        document.getElementById('demand-search').addEventListener('input', updateDemandTable);

        // Sortable demand table headers
        document.querySelectorAll('#demand-table thead .sortable-th').forEach(th => {
            th.addEventListener('click', function () {
                const key = this.dataset.sort;
                if (State.demandSortKey === key) {
                    State.demandSortAsc = !State.demandSortAsc;
                } else {
                    State.demandSortKey = key;
                    State.demandSortAsc = true;
                }
                updateDemandTable();
            });
        });

        // Arc sort buttons
        document.querySelectorAll('#arc-sort-bar .btn-sort').forEach(btn => {
            btn.addEventListener('click', function () {
                const key = this.dataset.sort;
                if (State.arcSortKey === key) {
                    State.arcSortAsc = !State.arcSortAsc;
                } else {
                    State.arcSortKey = key;
                    State.arcSortAsc = (key === 'load') ? false : true;
                }
                // Update button labels
                document.querySelectorAll('#arc-sort-bar .btn-sort').forEach(b => {
                    b.classList.remove('active');
                    b.textContent = b.textContent.replace(/ [▲▼]/, '');
                });
                this.classList.add('active');
                this.textContent += State.arcSortAsc ? ' ▲' : ' ▼';
                updateCongestedArcsList();
            });
        });

        // Header checkbox: select/deselect all
        document.getElementById('chk-select-all').addEventListener('change', function () {
            const snap = getCurrentSnapshot();
            if (!snap) return;
            if (this.checked) {
                snap.demands.forEach(d => State.selectedDemandIds.add(d.demand_id));
            } else {
                State.selectedDemandIds.clear();
            }
            updateDemandTable();
        });

        // Multi-select buttons
        document.getElementById('btn-select-all-demands').addEventListener('click', function () {
            const snap = getCurrentSnapshot();
            if (!snap) return;
            snap.demands.forEach(d => State.selectedDemandIds.add(d.demand_id));
            updateDemandTable();
        });

        document.getElementById('btn-clear-selection').addEventListener('click', function () {
            State.selectedDemandIds.clear();
            if (State.showingMultiRoutes) {
                State.showingMultiRoutes = false;
                clearSelection();
            }
            updateDemandTable();
        });

        document.getElementById('btn-select-congested').addEventListener('click', function () {
            const snap = getCurrentSnapshot();
            if (!snap) return;
            State.selectedDemandIds.clear();
            snap.demands.filter(d => d.is_candidate).forEach(d => State.selectedDemandIds.add(d.demand_id));
            updateDemandTable();
        });

        document.getElementById('btn-show-selected-routes').addEventListener('click', function () {
            if (State.selectedDemandIds.size === 0) return;
            if (!State.graph) buildGraph();
            highlightMultipleDemandRoutes();
        });

        // Instance file loading modal
        document.getElementById('btn-load-instance').addEventListener('click', function () {
            document.getElementById('instance-modal').classList.remove('hidden');
        });
        document.getElementById('btn-inst-cancel').addEventListener('click', function () {
            document.getElementById('instance-modal').classList.add('hidden');
        });
        document.getElementById('modal-overlay').addEventListener('click', function () {
            document.getElementById('instance-modal').classList.add('hidden');
        });
        document.getElementById('btn-inst-load').addEventListener('click', function () {
            const netFile = document.getElementById('inst-net-file').files[0];
            const tmFile = document.getElementById('inst-tm-file').files[0];
            const scenarioFile = document.getElementById('inst-scenario-file').files[0];
            const solutionFile = document.getElementById('inst-solution-file').files[0] || null;

            if (!netFile || !tmFile || !scenarioFile) {
                const errorEl = document.getElementById('inst-load-error');
                errorEl.textContent = 'Please select all three required files (net, tm, scenario).';
                errorEl.classList.remove('hidden');
                return;
            }

            loadFromInstanceFiles(netFile, tmFile, scenarioFile, solutionFile);
        });

        // Waypoint editor buttons
        document.getElementById('btn-add-waypoint').addEventListener('click', function () {
            const select = document.getElementById('waypoint-node-select');
            const nodeId = parseInt(select.value);
            if (isNaN(nodeId)) return;

            const snap = getCurrentSnapshot();
            if (!snap || State.selectedDemandId === null) return;
            const demand = snap.demands.find(d => d.demand_id === State.selectedDemandId);
            if (!demand) return;

            addWaypoint(demand, nodeId);
            select.value = '';
        });

        document.getElementById('btn-apply-waypoints').addEventListener('click', applyWaypointChanges);
        document.getElementById('btn-reset-waypoints').addEventListener('click', resetWaypoints);

        // Also allow clicking a node in graph to add as waypoint when editing
        // (integrated into existing node tap handler)

        // Demand actions
        document.getElementById('btn-highlight-demand').addEventListener('click', function () {
            if (State.selectedDemandId === null) return;
            if (State.showingFlow) exitFlowOverlay();
            const snap = getCurrentSnapshot();
            if (!snap) return;
            const demand = snap.demands.find(d => d.demand_id === State.selectedDemandId);
            if (demand) highlightDemandRoute(demand);
        });

        document.getElementById('btn-show-flow').addEventListener('click', function () {
            if (State.selectedDemandId === null) return;
            const snap = getCurrentSnapshot();
            if (!snap) return;
            const demand = snap.demands.find(d => d.demand_id === State.selectedDemandId);
            if (!demand) return;
            if (!State.graph) buildGraph();
            showDemandFlowOverlay(demand);
        });

        document.getElementById('btn-show-segments').addEventListener('click', function () {
            if (State.selectedDemandId === null) return;
            const snap = getCurrentSnapshot();
            if (!snap) return;
            const demand = snap.demands.find(d => d.demand_id === State.selectedDemandId);
            if (demand) {
                const pathNodes = [demand.source, ...demand.waypoints, demand.target];
                const segments = [];
                for (let i = 0; i < pathNodes.length - 1; i++) {
                    segments.push(`v${pathNodes[i]} → v${pathNodes[i + 1]}`);
                }
                alert('Segments for demand ' + demand.demand_id + ':\n' + segments.join('\n'));
            }
        });

        // Tabs
        document.querySelectorAll('#right-panel .tab').forEach(tab => {
            tab.addEventListener('click', function () {
                switchTab(this.dataset.tab);
            });
        });

        // Keyboard shortcuts
        document.addEventListener('keydown', function (e) {
            if (e.target.tagName === 'INPUT') return;
            switch (e.key) {
                case 'ArrowRight': nextTimeSlot(); break;
                case 'ArrowLeft': prevTimeSlot(); break;
                case 'ArrowUp': nextIteration(); break;
                case 'ArrowDown': prevIteration(); break;
                case 'f': State.cy && State.cy.fit(); break;
                case 'Escape': clearSelection(); break;
            }
        });
    }

    // ========================================================================
    // DEMO DATA GENERATOR
    // ========================================================================
    function generateDemoData() {
        const numNodes = 12;
        const numArcs = 30;
        const numDemands = 20;

        const nodes = [];
        for (let i = 0; i < numNodes; i++) {
            nodes.push({ id: i, name: 'v' + i });
        }

        const arcs = [];
        const added = new Set();
        let arcId = 0;
        for (let i = 0; i < numNodes; i++) {
            for (let j = 0; j < numNodes; j++) {
                if (i !== j && arcId < numArcs) {
                    const key = i + '-' + j;
                    if (!added.has(key)) {
                        arcs.push({
                            id: arcId,
                            from: i, to: j,
                            metric: 100 + Math.floor(Math.random() * 400),
                            capacity: 100 + Math.floor(Math.random() * 400)
                        });
                        added.add(key);
                        arcId++;
                    }
                }
            }
        }

        const demands = [];
        for (let i = 0; i < numDemands; i++) {
            const s = Math.floor(Math.random() * numNodes);
            let t = Math.floor(Math.random() * numNodes);
            while (t === s) t = Math.floor(Math.random() * numNodes);
            demands.push({
                demand_id: i, source: s, target: t,
                volume: Math.round(Math.random() * 200 * 100) / 100,
                waypoints: Math.random() > 0.5 ? [Math.floor(Math.random() * numNodes)] : [],
                contribution: Math.random() > 0.7 ? Math.round(Math.random() * 100) : 0,
                is_candidate: Math.random() > 0.7,
            });
        }

        const arcLoads = arcs.map(a => ({
            arc_id: a.id, from: a.from, to: a.to,
            load: Math.random() * 1.2,
            capacity: a.capacity,
            flow: Math.random() * a.capacity
        }));

        const topLoads = arcLoads.map(a => a.load).sort((a, b) => b - a).slice(0, 10);
        const mlu = topLoads[0];
        const mostCongestedId = arcLoads.reduce((best, a) => a.load > (best ? best.load : 0) ? a : best, null).arc_id;

        const snapshots = [
            {
                iteration: 0, time: 0,
                objective: { mlu: mlu, top_loads: topLoads },
                budget: { used: 0, allowed: 0 },
                violations: [],
                intervention_arcs: [],
                most_congested_arc_id: mostCongestedId,
                arc_loads: arcLoads,
                demands: demands,
                move: { type: 'initial', demand_id: -1, details: 'Initial solution' }
            },
            {
                iteration: 0, time: 1,
                objective: { mlu: mlu * 0.95, top_loads: topLoads.map(l => l * 0.95) },
                budget: { used: 10, allowed: 50 },
                violations: [],
                intervention_arcs: arcs.length > 5 ? [arcs[5].id] : [],
                most_congested_arc_id: mostCongestedId,
                arc_loads: arcLoads.map(a => ({ ...a, load: a.load * 0.95, flow: a.flow * 0.95 })),
                demands: demands,
                move: { type: 'final_solution', demand_id: -1, details: 'Final solution at time 1' }
            }
        ];

        return {
            instance_name: 'demo',
            num_nodes: numNodes,
            num_arcs: arcs.length,
            num_demands: numDemands,
            num_time_slots: 2,
            max_segments: 6,
            nodes: nodes,
            arcs: arcs,
            snapshots: snapshots
        };
    }

    // ========================================================================
    // ROUTING ENGINE — Dijkstra + Flow Computation
    // ========================================================================

    function buildGraph() {
        if (!State.data) return;
        // adjacency list: graph[fromNode] = [ { to, arcId, metric } ]
        const g = {};
        State.data.nodes.forEach(n => { g[n.id] = []; });
        State.data.arcs.forEach(a => {
            g[a.from].push({ to: a.to, arcId: a.id, metric: a.metric });
        });
        State.graph = g;
    }

    function dijkstra(source, target) {
        // Returns { path: [nodeIds], arcs: [arcIds], dist: number } or null
        const g = State.graph;
        if (!g) return null;

        const dist = {};
        const prev = {};
        const prevArc = {};
        const visited = new Set();

        State.data.nodes.forEach(n => { dist[n.id] = Infinity; });
        dist[source] = 0;

        // Simple priority queue with array (fine for small graphs)
        const pq = [{ node: source, d: 0 }];

        while (pq.length > 0) {
            // Extract min
            pq.sort((a, b) => a.d - b.d);
            const { node: u, d: du } = pq.shift();

            if (visited.has(u)) continue;
            visited.add(u);

            if (u === target) break;

            if (!g[u]) continue;
            for (const edge of g[u]) {
                const newDist = du + edge.metric;
                if (newDist < dist[edge.to]) {
                    dist[edge.to] = newDist;
                    prev[edge.to] = u;
                    prevArc[edge.to] = edge.arcId;
                    pq.push({ node: edge.to, d: newDist });
                }
            }
        }

        if (dist[target] === Infinity) return null;

        // Reconstruct path
        const path = [];
        const arcs = [];
        let cur = target;
        while (cur !== source) {
            path.unshift(cur);
            arcs.unshift(prevArc[cur]);
            cur = prev[cur];
        }
        path.unshift(source);

        return { path, arcs, dist: dist[target] };
    }

    /**
     * Find ALL arcs that lie on ANY shortest path from source to target (ECMP).
     * @param {number} source - Source node ID
     * @param {number} target - Target node ID
     * @param {Set} [excludeNodes] - Optional set of node IDs to exclude from paths (no-revisit)
     * Returns { shortestArcs: Set<arcId>, shortestDist: number, primaryPath: [arcIds] }
     */
    function dijkstraAllShortestPaths(source, target, excludeNodes) {
        const g = State.graph;
        if (!g) return null;

        const dist = {};
        const prevNodes = {};  // prevNodes[v] = list of predecessors on shortest paths
        const prevArcs = {};   // prevArcs[v] = list of arc IDs from predecessors
        const visited = new Set();

        State.data.nodes.forEach(n => {
            dist[n.id] = Infinity;
            prevNodes[n.id] = [];
            prevArcs[n.id] = [];
        });
        dist[source] = 0;

        const pq = [{ node: source, d: 0 }];

        while (pq.length > 0) {
            pq.sort((a, b) => a.d - b.d);
            const { node: u, d: du } = pq.shift();

            if (visited.has(u)) continue;
            visited.add(u);

            if (!g[u]) continue;
            for (const edge of g[u]) {
                // Skip excluded nodes (no-revisit), but always allow the target
                if (excludeNodes && excludeNodes.has(edge.to) && edge.to !== target) continue;

                const newDist = du + edge.metric;
                if (newDist < dist[edge.to]) {
                    dist[edge.to] = newDist;
                    prevNodes[edge.to] = [u];
                    prevArcs[edge.to] = [edge.arcId];
                    pq.push({ node: edge.to, d: newDist });
                } else if (Math.abs(newDist - dist[edge.to]) < 1e-9) {
                    // Equal-cost path — add as another predecessor
                    prevNodes[edge.to].push(u);
                    prevArcs[edge.to].push(edge.arcId);
                }
            }
        }

        if (dist[target] === Infinity) return null;

        // BFS backwards from target to collect all arcs on shortest paths
        const shortestArcs = new Set();
        const queue = [target];
        const visitedBack = new Set([target]);

        while (queue.length > 0) {
            const v = queue.shift();
            for (let i = 0; i < prevNodes[v].length; i++) {
                shortestArcs.add(prevArcs[v][i]);
                const pred = prevNodes[v][i];
                if (!visitedBack.has(pred)) {
                    visitedBack.add(pred);
                    queue.push(pred);
                }
            }
        }

        // Also get a single primary path for reference
        const primaryResult = dijkstra(source, target);
        const primaryArcs = primaryResult ? primaryResult.arcs : [];

        return { shortestArcs, shortestDist: dist[target], primaryArcs };
    }

    /**
     * Compute ECMP flow distribution for a single segment (source -> target).
     * Traffic of `volume` is split equally at each ECMP branching point.
     * @param {number} source - Source node ID
     * @param {number} target - Target node ID
     * @param {number} volume - Traffic volume
     * @param {Set} [excludeNodes] - Optional set of node IDs to exclude from paths (no-revisit)
     * Returns Map<arcId, flowAmount> for all arcs carrying flow.
     */
    function computeEcmpFlow(source, target, volume, excludeNodes) {
        const g = State.graph;
        if (!g) return new Map();

        // Run Dijkstra to get shortest distances and predecessors
        const dist = {};
        const succNodes = {};  // succNodes[v] = list of { to, arcId } on shortest paths forward
        const visited = new Set();

        State.data.nodes.forEach(n => {
            dist[n.id] = Infinity;
            succNodes[n.id] = [];
        });
        dist[source] = 0;

        const pq = [{ node: source, d: 0 }];
        while (pq.length > 0) {
            pq.sort((a, b) => a.d - b.d);
            const { node: u, d: du } = pq.shift();
            if (visited.has(u)) continue;
            visited.add(u);
            if (!g[u]) continue;
            for (const edge of g[u]) {
                // Skip excluded nodes (no-revisit), but always allow the target
                if (excludeNodes && excludeNodes.has(edge.to) && edge.to !== target) continue;

                const newDist = du + edge.metric;
                if (newDist < dist[edge.to]) {
                    dist[edge.to] = newDist;
                    pq.push({ node: edge.to, d: newDist });
                } else if (Math.abs(newDist - dist[edge.to]) > 1e-9) {
                    continue;
                }
                // newDist equals dist[edge.to] — don't add successor yet
            }
        }

        if (dist[target] === Infinity) return new Map();

        // Build the shortest-path DAG: for each node on the DAG,
        // collect successors (forward edges that are on some shortest path to target)
        // We need backward predecessors to build the forward DAG
        const prevNodes = {};
        const prevArcs = {};
        State.data.nodes.forEach(n => {
            prevNodes[n.id] = [];
            prevArcs[n.id] = [];
        });

        // Re-scan all edges to find shortest-path DAG edges
        State.data.arcs.forEach(a => {
            if (dist[a.from] !== Infinity && dist[a.to] !== Infinity) {
                if (Math.abs(dist[a.from] + a.metric - dist[a.to]) < 1e-9) {
                    prevNodes[a.to].push(a.from);
                    prevArcs[a.to].push(a.id);
                }
            }
        });

        // Find all nodes reachable on shortest paths from source to target
        // BFS backwards from target
        const onDag = new Set();
        const bfsQ = [target];
        onDag.add(target);
        while (bfsQ.length > 0) {
            const v = bfsQ.shift();
            for (let i = 0; i < prevNodes[v].length; i++) {
                const pred = prevNodes[v][i];
                if (!onDag.has(pred)) {
                    onDag.add(pred);
                    bfsQ.push(pred);
                }
            }
        }

        if (!onDag.has(source)) return new Map();

        // Build forward DAG restricted to nodes on DAG
        const fwdEdges = {};  // fwdEdges[u] = [{to, arcId}]
        onDag.forEach(n => { fwdEdges[n] = []; });

        State.data.arcs.forEach(a => {
            if (onDag.has(a.from) && onDag.has(a.to)) {
                if (Math.abs(dist[a.from] + a.metric - dist[a.to]) < 1e-9) {
                    fwdEdges[a.from].push({ to: a.to, arcId: a.id });
                }
            }
        });

        // Topological sort of the DAG (by distance from source)
        const topoOrder = Array.from(onDag).sort((a, b) => dist[a] - dist[b]);

        // Forward pass: split flow equally at each branching point
        const nodeFlow = {};
        onDag.forEach(n => { nodeFlow[n] = 0; });
        nodeFlow[source] = volume;

        const arcFlow = new Map();

        for (const u of topoOrder) {
            if (u === target) continue;
            const outEdges = fwdEdges[u];
            if (!outEdges || outEdges.length === 0) continue;

            const flowPerEdge = nodeFlow[u] / outEdges.length;
            for (const e of outEdges) {
                arcFlow.set(e.arcId, (arcFlow.get(e.arcId) || 0) + flowPerEdge);
                nodeFlow[e.to] = (nodeFlow[e.to] || 0) + flowPerEdge;
            }
        }

        return arcFlow;
    }

    /**
     * Compute the full ECMP flow for a demand across all its segments.
     * Returns { arcFlows: Map<arcId, flow>, nodeFlows: Map<nodeId, flow>, totalDist: number }
     */
    function computeDemandEcmpFlow(demand) {
        if (!State.graph) return null;

        const segmentEndpoints = [demand.source, ...demand.waypoints, demand.target];
        const arcFlows = new Map();
        const nodeFlows = new Map();
        let totalDist = 0;
        const visitedNodes = State.noRevisit ? new Set([demand.source]) : null;

        // Each segment carries the full volume (SR forwarding: each segment independently)
        for (let i = 0; i < segmentEndpoints.length - 1; i++) {
            const from = segmentEndpoints[i];
            const to = segmentEndpoints[i + 1];

            const segFlow = computeEcmpFlow(from, to, demand.volume, visitedNodes);

            // Merge arc flows
            segFlow.forEach((flow, arcId) => {
                arcFlows.set(arcId, (arcFlows.get(arcId) || 0) + flow);
            });

            // Track visited nodes for no-revisit
            if (visitedNodes) {
                segFlow.forEach((_, arcId) => {
                    const arc = State.data.arcs.find(a => a.id === arcId);
                    if (arc) { visitedNodes.add(arc.from); visitedNodes.add(arc.to); }
                });
            }

            // Track intermediate node flows
            const result = dijkstra(from, to);
            if (result) totalDist += result.dist;
        }

        // Compute node flows: sum of incoming flow for each node
        arcFlows.forEach((flow, arcId) => {
            const arc = State.data.arcs.find(a => a.id === arcId);
            if (arc) {
                nodeFlows.set(arc.to, (nodeFlows.get(arc.to) || 0) + flow);
                // Also count outgoing from source
                if (!nodeFlows.has(arc.from)) nodeFlows.set(arc.from, 0);
            }
        });

        return { arcFlows, nodeFlows, totalDist };
    }

    /**
     * Show ECMP flow overlay on the graph for the given demand.
     * Dims all non-participating arcs, colors flow arcs with width proportional
     * to flow amount, and labels each arc with the flow value.
     */
    function showDemandFlowOverlay(demand) {
        if (!State.cy || !State.graph) return;

        const flowResult = computeDemandEcmpFlow(demand);
        if (!flowResult) return;

        State.showingFlow = true;
        State.flowDemandId = demand.demand_id;

        const { arcFlows, nodeFlows } = flowResult;
        const maxFlow = Math.max(...arcFlows.values(), 0.001);

        // First: dim everything + clear highlights
        State.cy.nodes().removeClass('highlighted source-node target-node waypoint-node flow-node');
        State.cy.edges().removeClass('highlighted-route ecmp-path segment-0 segment-1 segment-2 segment-3 segment-4 segment-5 flow-edge flow-edge-heavy flow-edge-medium flow-edge-light');

        State.cy.edges().addClass('flow-dimmed');
        State.cy.nodes().addClass('flow-dimmed');

        // Highlight source, target, waypoints
        const srcNode = State.cy.getElementById('n' + demand.source);
        const tgtNode = State.cy.getElementById('n' + demand.target);
        if (srcNode.length) { srcNode.removeClass('flow-dimmed'); srcNode.addClass('source-node'); }
        if (tgtNode.length) { tgtNode.removeClass('flow-dimmed'); tgtNode.addClass('target-node'); }

        demand.waypoints.forEach(wp => {
            const wpNode = State.cy.getElementById('n' + wp);
            if (wpNode.length) { wpNode.removeClass('flow-dimmed'); wpNode.addClass('waypoint-node'); }
        });

        // Color arcs by flow, set labels, set widths
        arcFlows.forEach((flow, arcId) => {
            const edge = State.cy.getElementById('e' + arcId);
            if (!edge.length) return;

            edge.removeClass('flow-dimmed');
            edge.addClass('flow-edge');

            // Width proportional to flow fraction
            const fraction = flow / maxFlow;
            const width = Math.max(2, fraction * 10);
            edge.style('width', width);

            // Color by flow intensity
            if (fraction > 0.7) {
                edge.style('line-color', '#7aa2f7');
                edge.style('target-arrow-color', '#7aa2f7');
            } else if (fraction > 0.3) {
                edge.style('line-color', '#73daca');
                edge.style('target-arrow-color', '#73daca');
            } else {
                edge.style('line-color', '#9ece6a');
                edge.style('target-arrow-color', '#9ece6a');
            }

            // Show flow label
            edge.data('label', fmt(flow, 2));

            // Un-dim the endpoints
            const srcId = edge.data('source');
            const tgtId = edge.data('target');
            State.cy.getElementById(srcId).removeClass('flow-dimmed').addClass('flow-node');
            State.cy.getElementById(tgtId).removeClass('flow-dimmed').addClass('flow-node');
        });

        // Update styles so labels show on flow edges
        State.cy.style().fromJson(getCytoscapeStyle()).update();
        // Re-apply flow styles after style reset
        State.cy.edges().forEach(edge => {
            if (arcFlows.has(edge.data('arcId'))) {
                const flow = arcFlows.get(edge.data('arcId'));
                const fraction = flow / maxFlow;
                const width = Math.max(2, fraction * 10);
                edge.style('width', width);
                if (fraction > 0.7) {
                    edge.style('line-color', '#7aa2f7');
                    edge.style('target-arrow-color', '#7aa2f7');
                } else if (fraction > 0.3) {
                    edge.style('line-color', '#73daca');
                    edge.style('target-arrow-color', '#73daca');
                } else {
                    edge.style('line-color', '#9ece6a');
                    edge.style('target-arrow-color', '#9ece6a');
                }
                edge.data('label', fmt(flow, 2));
            }
        });

        // Build flow info panel
        const detail = document.getElementById('demand-detail-content');
        const segColors = ['#7aa2f7', '#9ece6a', '#ff9e64', '#e0af68', '#bb9af7', '#f7768e'];
        const segmentEndpoints = [demand.source, ...demand.waypoints, demand.target];

        let flowHtml = '<div class="flow-summary">';
        flowHtml += `<div class="flow-header">ECMP Flow Distribution — Demand ${demand.demand_id}</div>`;
        flowHtml += `<div class="flow-row"><span class="flow-label">Volume:</span><span class="flow-val">${fmt(demand.volume, 2)}</span></div>`;
        flowHtml += `<div class="flow-row"><span class="flow-label">Total arcs carrying flow:</span><span class="flow-val">${arcFlows.size}</span></div>`;
        flowHtml += `<div class="flow-row"><span class="flow-label">Path:</span><span class="flow-val">v${demand.source}${demand.waypoints.map(w => ' → v' + w).join('')} → v${demand.target}</span></div>`;

        // Per-segment breakdown
        const visitedNodesOverlay = State.noRevisit ? new Set([demand.source]) : null;
        for (let i = 0; i < segmentEndpoints.length - 1; i++) {
            const from = segmentEndpoints[i];
            const to = segmentEndpoints[i + 1];
            const color = segColors[i % segColors.length];
            const segFlow = computeEcmpFlow(from, to, demand.volume, visitedNodesOverlay);
            const numArcs = segFlow.size;

            // Track visited nodes for no-revisit
            if (visitedNodesOverlay) {
                segFlow.forEach((_, aid) => {
                    const arc = State.data.arcs.find(a => a.id === aid);
                    if (arc) { visitedNodesOverlay.add(arc.from); visitedNodesOverlay.add(arc.to); }
                });
            }

            // Count branching points: nodes with >1 outgoing arc in the flow DAG
            const outCount = {};
            segFlow.forEach((fl, arcId) => {
                const arc = State.data.arcs.find(a => a.id === arcId);
                if (arc) outCount[arc.from] = (outCount[arc.from] || 0) + 1;
            });
            const branchingNodes = Object.values(outCount).filter(c => c > 1).length;

            flowHtml += `<div class="flow-segment">
                <span class="segment-indicator" style="background:${color}"></span>
                <span class="flow-seg-title">Segment ${i + 1}: v${from} → v${to}</span>
                <div class="flow-seg-details">
                    <span>${numArcs} arcs</span>
                    <span>${branchingNodes} ECMP split${branchingNodes !== 1 ? 's' : ''}</span>
                </div>
            </div>`;

            // Show top arcs by flow
            const sortedArcs = [...segFlow.entries()].sort((a, b) => b[1] - a[1]);
            if (sortedArcs.length > 0) {
                flowHtml += '<div class="flow-arc-list">';
                sortedArcs.slice(0, 8).forEach(([arcId, fl]) => {
                    const arc = State.data.arcs.find(a => a.id === arcId);
                    const pct = ((fl / demand.volume) * 100).toFixed(1);
                    if (arc) {
                        flowHtml += `<div class="flow-arc-row">
                            <span class="flow-arc-path">v${arc.from}→v${arc.to}</span>
                            <span class="flow-arc-bar"><span class="flow-arc-fill" style="width:${pct}%;background:${color}"></span></span>
                            <span class="flow-arc-val">${fmt(fl, 2)} (${pct}%)</span>
                        </div>`;
                    }
                });
                if (sortedArcs.length > 8) {
                    flowHtml += `<div class="flow-arc-more">... and ${sortedArcs.length - 8} more arcs</div>`;
                }
                flowHtml += '</div>';
            }
        }

        flowHtml += `<div class="flow-actions">
            <button class="btn btn-sm btn-secondary" id="btn-exit-flow">Exit Flow View</button>
        </div>`;
        flowHtml += '</div>';

        detail.innerHTML = flowHtml;

        // Bind exit button
        document.getElementById('btn-exit-flow').addEventListener('click', function () {
            exitFlowOverlay();
            if (State.selectedDemandId !== null) selectDemand(State.selectedDemandId);
        });
    }

    /**
     * Exit flow overlay mode — restore normal graph appearance.
     */
    function exitFlowOverlay() {
        State.showingFlow = false;
        State.flowDemandId = null;

        if (State.cy) {
            State.cy.nodes().removeClass('flow-dimmed flow-node');
            State.cy.edges().removeClass('flow-dimmed flow-edge');
            State.cy.style().fromJson(getCytoscapeStyle()).update();
        }

        updateGraphLoads();
    }

    function computePathForDemand(source, target, waypoints) {
        // Segment routing: source -> wp1 -> wp2 -> ... -> target
        // Each segment follows shortest path (by IGP metric)
        const segmentEndpoints = [source, ...waypoints, target];
        const fullPath = [];
        const fullArcs = [];

        for (let i = 0; i < segmentEndpoints.length - 1; i++) {
            const result = dijkstra(segmentEndpoints[i], segmentEndpoints[i + 1]);
            if (!result) return null; // no path exists

            // Append nodes (skip first node of subsequent segments to avoid duplicates)
            if (i === 0) {
                fullPath.push(...result.path);
            } else {
                fullPath.push(...result.path.slice(1));
            }
            fullArcs.push(...result.arcs);
        }

        return { path: fullPath, arcs: fullArcs };
    }

    function recomputeAllLoads(modifiedDemandId, newWaypoints) {
        // Recompute arc flows from scratch for the current snapshot,
        // using newWaypoints for the modified demand.
        const snap = getCurrentSnapshot();
        if (!snap || !State.graph) return null;

        // Build capacity map
        const capMap = {};
        State.data.arcs.forEach(a => { capMap[a.id] = a.capacity; });

        // Arc flow accumulator
        const flowMap = {};
        State.data.arcs.forEach(a => { flowMap[a.id] = 0; });

        // Intervention set - arcs with zero capacity
        const interventionSet = new Set(snap.intervention_arcs || []);

        let modifiedDemandPath = null;

        for (const d of snap.demands) {
            const wps = (d.demand_id === modifiedDemandId) ? newWaypoints : d.waypoints;
            const result = computePathForDemand(d.source, d.target, wps);
            if (!result) continue; // skip unreachable

            if (d.demand_id === modifiedDemandId) {
                modifiedDemandPath = result;
            }

            // Accumulate flow (simple: all volume on shortest path, no ECMP split)
            for (const arcId of result.arcs) {
                if (!interventionSet.has(arcId)) {
                    flowMap[arcId] += d.volume;
                }
            }
        }

        // Compute loads
        const newArcLoads = [];
        let mlu = 0;
        State.data.arcs.forEach(a => {
            const flow = flowMap[a.id] || 0;
            const cap = capMap[a.id] || 1;
            const load = interventionSet.has(a.id) ? 0 : flow / cap;
            if (load > mlu) mlu = load;
            newArcLoads.push({
                arc_id: a.id,
                from: a.from,
                to: a.to,
                load: load,
                capacity: cap,
                flow: flow,
            });
        });

        const topLoads = newArcLoads.map(a => a.load).sort((a, b) => b - a).slice(0, 10);

        return { arcLoads: newArcLoads, mlu, topLoads, modifiedDemandPath };
    }

    // ========================================================================
    // WAYPOINT EDITING UI
    // ========================================================================

    function showWaypointEditor(demand) {
        State.editingWaypoints = [...demand.waypoints];
        State.originalWaypoints = [...demand.waypoints];

        const editor = document.getElementById('waypoint-editor');
        editor.classList.remove('hidden');

        // Populate node select dropdown (exclude source and target)
        const select = document.getElementById('waypoint-node-select');
        select.innerHTML = '<option value="">— select node —</option>';
        State.data.nodes
            .slice()
            .sort((a, b) => a.id - b.id)
            .forEach(n => {
                if (n.id !== demand.source && n.id !== demand.target) {
                    const opt = document.createElement('option');
                    opt.value = n.id;
                    opt.textContent = n.name + ' (id:' + n.id + ')';
                    select.appendChild(opt);
                }
            });

        updateWaypointList(demand);

        // Hide result
        const resultEl = document.getElementById('waypoint-result');
        resultEl.classList.add('hidden');
    }

    function hideWaypointEditor() {
        document.getElementById('waypoint-editor').classList.add('hidden');
        State.editingWaypoints = null;
        State.originalWaypoints = null;
    }

    function updateWaypointList(demand) {
        const listEl = document.getElementById('waypoint-list');
        listEl.innerHTML = '';

        if (!State.editingWaypoints || State.editingWaypoints.length === 0) {
            listEl.innerHTML = '<div class="wp-empty">No waypoints (direct shortest path)</div>';
            return;
        }

        // Show full segment chain
        const chain = [demand.source, ...State.editingWaypoints, demand.target];

        State.editingWaypoints.forEach((wp, idx) => {
            const item = document.createElement('div');
            item.className = 'wp-item';

            // Find node name
            const node = State.data.nodes.find(n => n.id === wp);
            const name = node ? node.name : 'v' + wp;

            item.innerHTML = `
                <span class="wp-index">${idx + 1}</span>
                <span class="wp-name">${name}</span>
                <div class="wp-item-actions">
                    ${idx > 0 ? '<button class="btn-wp-move" data-dir="up" data-idx="' + idx + '" title="Move up">▲</button>' : ''}
                    ${idx < State.editingWaypoints.length - 1 ? '<button class="btn-wp-move" data-dir="down" data-idx="' + idx + '" title="Move down">▼</button>' : ''}
                    <button class="btn-wp-remove" data-idx="${idx}" title="Remove waypoint">✕</button>
                </div>
            `;
            listEl.appendChild(item);
        });

        // Bind remove/move buttons
        listEl.querySelectorAll('.btn-wp-remove').forEach(btn => {
            btn.addEventListener('click', function () {
                const idx = parseInt(this.dataset.idx);
                State.editingWaypoints.splice(idx, 1);
                updateWaypointList(demand);
                previewWaypointRoute(demand);
            });
        });

        listEl.querySelectorAll('.btn-wp-move').forEach(btn => {
            btn.addEventListener('click', function () {
                const idx = parseInt(this.dataset.idx);
                const dir = this.dataset.dir;
                if (dir === 'up' && idx > 0) {
                    [State.editingWaypoints[idx], State.editingWaypoints[idx - 1]] =
                        [State.editingWaypoints[idx - 1], State.editingWaypoints[idx]];
                } else if (dir === 'down' && idx < State.editingWaypoints.length - 1) {
                    [State.editingWaypoints[idx], State.editingWaypoints[idx + 1]] =
                        [State.editingWaypoints[idx + 1], State.editingWaypoints[idx]];
                }
                updateWaypointList(demand);
                previewWaypointRoute(demand);
            });
        });
    }

    function addWaypoint(demand, nodeId) {
        if (State.editingWaypoints === null) return;
        const maxSeg = State.data.max_segments;
        // Number of segments = number of waypoints + 1, must be <= max_segments
        if (State.editingWaypoints.length + 1 >= maxSeg) {
            alert('Cannot add more waypoints: max segments = ' + maxSeg + ' (current waypoints: ' + State.editingWaypoints.length + ')');
            return;
        }
        if (nodeId === demand.source || nodeId === demand.target) {
            return; // can't add source/target as waypoint
        }
        State.editingWaypoints.push(nodeId);
        updateWaypointList(demand);
        previewWaypointRoute(demand);
    }

    function previewWaypointRoute(demand) {
        if (!State.cy || !State.graph || !State.editingWaypoints) return;

        // Clear previous highlights
        State.cy.nodes().removeClass('highlighted source-node target-node waypoint-node');
        State.cy.edges().removeClass('highlighted-route ecmp-path segment-0 segment-1 segment-2 segment-3 segment-4 segment-5');

        // Highlight source/target
        const srcNode = State.cy.getElementById('n' + demand.source);
        const tgtNode = State.cy.getElementById('n' + demand.target);
        if (srcNode.length) srcNode.addClass('source-node');
        if (tgtNode.length) tgtNode.addClass('target-node');

        // Highlight waypoints
        State.editingWaypoints.forEach(wp => {
            const wpNode = State.cy.getElementById('n' + wp);
            if (wpNode.length) wpNode.addClass('waypoint-node');
        });

        // Compute and show ECMP paths per segment
        const segmentEndpoints = [demand.source, ...State.editingWaypoints, demand.target];
        const segColors = ['segment-0', 'segment-1', 'segment-2', 'segment-3', 'segment-4', 'segment-5'];
        const visitedNodesPreview = State.noRevisit ? new Set([demand.source]) : null;

        for (let i = 0; i < segmentEndpoints.length - 1; i++) {
            const from = segmentEndpoints[i];
            const to = segmentEndpoints[i + 1];
            const segClass = segColors[i % segColors.length];

            const ecmpResult = dijkstraAllShortestPaths(from, to, visitedNodesPreview);
            if (ecmpResult) {
                const primarySet = new Set(ecmpResult.primaryArcs);
                ecmpResult.shortestArcs.forEach(arcId => {
                    const edge = State.cy.getElementById('e' + arcId);
                    if (edge.length) {
                        edge.addClass(primarySet.has(arcId) ? segClass : 'ecmp-path');
                    }
                });

                // Track visited nodes for no-revisit
                if (visitedNodesPreview) {
                    ecmpResult.shortestArcs.forEach(aid => {
                        const arc = State.data.arcs.find(a => a.id === aid);
                        if (arc) { visitedNodesPreview.add(arc.from); visitedNodesPreview.add(arc.to); }
                    });
                }
            }
        }
    }

    function applyWaypointChanges() {
        if (State.selectedDemandId === null || !State.editingWaypoints) return;

        const snap = getCurrentSnapshot();
        if (!snap) return;

        const demand = snap.demands.find(d => d.demand_id === State.selectedDemandId);
        if (!demand) return;

        const resultEl = document.getElementById('waypoint-result');
        resultEl.classList.remove('hidden');
        resultEl.innerHTML = '<div class="computing">Computing new loads...</div>';

        // Build graph if not built
        if (!State.graph) buildGraph();

        // Compute old MLU
        const oldMlu = snap.objective.mlu;

        // Recompute loads
        const result = recomputeAllLoads(demand.demand_id, State.editingWaypoints);
        if (!result) {
            resultEl.innerHTML = '<div class="wp-error">Error: Could not compute path (unreachable node?)</div>';
            return;
        }

        const mluDiff = result.mlu - oldMlu;
        const mluColor = mluDiff < -0.001 ? 'var(--accent-green)' : (mluDiff > 0.001 ? 'var(--accent-red)' : 'var(--accent-blue)');
        const arrow = mluDiff < -0.001 ? '↓' : (mluDiff > 0.001 ? '↑' : '≈');

        resultEl.innerHTML = `
            <div class="wp-result-header">Recomputed Results</div>
            <div class="wp-result-row">
                <span>Old MLU:</span> <span>${fmt(oldMlu)}</span>
            </div>
            <div class="wp-result-row">
                <span>New MLU:</span> <span style="color:${mluColor};font-weight:700">${fmt(result.mlu)} ${arrow}</span>
            </div>
            <div class="wp-result-row">
                <span>Δ MLU:</span> <span style="color:${mluColor}">${mluDiff > 0 ? '+' : ''}${fmt(mluDiff)}</span>
            </div>
            <div class="wp-result-actions">
                <button class="btn btn-sm btn-primary" id="btn-confirm-waypoints">Confirm Changes</button>
                <button class="btn btn-sm btn-secondary" id="btn-cancel-waypoints">Cancel</button>
            </div>
        `;

        // Store result for confirmation
        State._pendingWaypointResult = result;
        State._pendingDemandId = demand.demand_id;

        document.getElementById('btn-confirm-waypoints').addEventListener('click', confirmWaypointChanges);
        document.getElementById('btn-cancel-waypoints').addEventListener('click', cancelWaypointChanges);
    }

    function confirmWaypointChanges() {
        if (!State._pendingWaypointResult || State._pendingDemandId === undefined) return;

        const snap = getCurrentSnapshot();
        if (!snap) return;

        const result = State._pendingWaypointResult;
        const demandId = State._pendingDemandId;

        // Update the snapshot data in place
        snap.arc_loads = result.arcLoads;
        snap.objective.mlu = result.mlu;
        snap.objective.top_loads = result.topLoads;

        // Update most congested arc
        let maxLoad = 0;
        let maxArcId = 0;
        result.arcLoads.forEach(al => {
            if (al.load > maxLoad) { maxLoad = al.load; maxArcId = al.arc_id; }
        });
        snap.most_congested_arc_id = maxArcId;

        // Update the demand's waypoints
        const demand = snap.demands.find(d => d.demand_id === demandId);
        if (demand) {
            demand.waypoints = [...State.editingWaypoints];
        }

        // Re-mark candidates: demands that route through the most congested arc
        if (State.graph) {
            const interventionSet = new Set(snap.intervention_arcs || []);
            snap.demands.forEach(d => {
                d.is_candidate = false;
                d.contribution = 0;
                if (d.volume > 0 && maxLoad > 0) {
                    const segEndpoints = [d.source, ...d.waypoints, d.target];
                    const visitedNodesCand = State.noRevisit ? new Set([d.source]) : null;
                    for (let i = 0; i < segEndpoints.length - 1; i++) {
                        const ecmp = dijkstraAllShortestPaths(segEndpoints[i], segEndpoints[i + 1], visitedNodesCand);
                        if (ecmp && ecmp.shortestArcs.has(maxArcId)) {
                            d.is_candidate = true;
                            d.contribution = d.volume;
                            break;
                        }
                        if (visitedNodesCand && ecmp) {
                            ecmp.shortestArcs.forEach(aid => {
                                const arc = State.data.arcs.find(a => a.id === aid);
                                if (arc) { visitedNodesCand.add(arc.from); visitedNodesCand.add(arc.to); }
                            });
                        }
                    }
                }
            });
        }

        // Update original
        State.originalWaypoints = [...State.editingWaypoints];

        // Clean up
        State._pendingWaypointResult = null;
        State._pendingDemandId = undefined;

        // Refresh the whole view
        updateAll();

        // Re-select the demand to refresh detail
        selectDemand(demandId);

        const resultEl = document.getElementById('waypoint-result');
        resultEl.innerHTML = '<div style="color:var(--accent-green)">✓ Changes applied!</div>';
        setTimeout(() => resultEl.classList.add('hidden'), 2000);
    }

    function cancelWaypointChanges() {
        // Revert to original
        if (State.originalWaypoints) {
            State.editingWaypoints = [...State.originalWaypoints];
        }

        State._pendingWaypointResult = null;
        State._pendingDemandId = undefined;

        const snap = getCurrentSnapshot();
        const demand = snap ? snap.demands.find(d => d.demand_id === State.selectedDemandId) : null;
        if (demand) {
            updateWaypointList(demand);
            previewWaypointRoute(demand);
        }

        document.getElementById('waypoint-result').classList.add('hidden');
    }

    function resetWaypoints() {
        if (!State.originalWaypoints) return;
        State.editingWaypoints = [...State.originalWaypoints];

        const snap = getCurrentSnapshot();
        const demand = snap ? snap.demands.find(d => d.demand_id === State.selectedDemandId) : null;
        if (demand) {
            updateWaypointList(demand);
            previewWaypointRoute(demand);
        }
        document.getElementById('waypoint-result').classList.add('hidden');
    }

    // ========================================================================
    // INIT
    // ========================================================================
    function init() {
        bindEvents();

        // Try auto-loading from URL param or default
        const params = new URLSearchParams(window.location.search);
        const dataFile = params.get('data');
        if (dataFile) {
            fetch(dataFile)
                .then(r => r.json())
                .then(data => loadData(data))
                .catch(err => console.warn('Could not load', dataFile, err));
        }

        // Auto-load instance files from URL parameters if provided
        const pathNet = params.get('path_net');
        const pathScenario = params.get('path_scenario');
        const pathTm = params.get('path_tm');
        const pathSolution = params.get('path_solution');

        if (pathNet && pathScenario && pathTm) {
            const filePromises = [
                fetch(pathNet).then(r => r.json()),
                fetch(pathTm).then(r => r.json()),
                fetch(pathScenario).then(r => r.json()),
                pathSolution ? fetch(pathSolution).then(r => r.json()) : Promise.resolve(null),
            ];
            Promise.all(filePromises)
                .then(([netData, tmData, scenarioData, solutionData]) => {
                    const solverState = buildSolverStateFromInstance(netData, tmData, scenarioData, solutionData);
                    loadData(solverState);
                })
                .catch(err => console.warn('Could not load instance files from URL params:', err));
        }
    }

    // Public API for inline event handlers
    window.TASRApp = {
        selectDemand: selectDemand,
        selectArc: selectArc,
        highlightMultipleDemandRoutes: highlightMultipleDemandRoutes,
        arcToggleDemand: arcToggleDemand,
        arcSelectAllDemands: arcSelectAllDemands,
        arcShowAllRoutes: arcShowAllRoutes,
        arcShowDemandFlow: arcShowDemandFlow,
        arcEditDemandWaypoints: arcEditDemandWaypoints,
    };

    document.addEventListener('DOMContentLoaded', init);
})();
