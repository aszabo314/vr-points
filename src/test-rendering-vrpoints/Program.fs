open System
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharp.Data.Adaptive
open FSharp.Data.Adaptive.Operators
open Aardvark.Application.Utilities
open Aardvark.Rendering.PointSet
open Aardvark.Base.Ag
open Aardvark.Geometry.Points
open Aardvark.Data.Points
open Aardvark.Data.Points.Import
open MySceneGraph


[<ReflectedDefinition>]
module PointSetShaders =
    open FShade
    open Aardvark.Rendering


    let constantColor (c : C4f) (v : Effects.Vertex) =
        let c = c.ToV4d()
        vertex {
            return { v with c = c }
        }


    let heatMapColors =
        let fromInt (i : int) =
            C4b(
                byte ((i >>> 16) &&& 0xFF),
                byte ((i >>> 8) &&& 0xFF),
                byte (i &&& 0xFF),
                255uy
            ).ToC4f().ToV4d()

        Array.map fromInt [|
            0x1639fa
            0x2050fa
            0x3275fb
            0x459afa
            0x55bdfb
            0x67e1fc
            0x72f9f4
            0x72f8d3
            0x72f7ad
            0x71f787
            0x71f55f
            0x70f538
            0x74f530
            0x86f631
            0x9ff633
            0xbbf735
            0xd9f938
            0xf7fa3b
            0xfae238
            0xf4be31
            0xf29c2d
            0xee7627
            0xec5223
            0xeb3b22
        |]

    let heat (tc : float) =
        let tc = clamp 0.0 1.0 tc
        let fid = tc * float heatMapColors.Length - 0.5

        let id = int (floor fid)
        if id < 0 then 
            heatMapColors.[0]
        elif id >= heatMapColors.Length - 1 then
            heatMapColors.[heatMapColors.Length - 1]
        else
            let c0 = heatMapColors.[id]
            let c1 = heatMapColors.[id + 1]
            let t = fid - float id
            (c0 * (1.0 - t) + c1 * t)


    type UniformScope with
        member x.PointVisualization : PointVisualization = x?PointVisualization
        member x.Overlay : V4d[] = x?StorageBuffer?Overlay
        member x.ModelTrafos : M44d[] = x?StorageBuffer?ModelTrafos
        member x.LeftModelViewTrafos : M44d[] = x?StorageBuffer?LeftModelViewTrafos
        member x.RightModelViewTrafos : M44d[] = x?StorageBuffer?RightModelViewTrafos
        member x.Scales : V4d[] = x?StorageBuffer?Scales

    type Vertex =
        {
            [<Position>] pos : V4d
            [<Normal>] n : V3d
            [<Semantic("Offsets")>] offset : V3d
        }


    let offset ( v : Vertex) =
        vertex {
            return  { v with pos = v.pos + V4d(v.offset, 0.0)}
        }
        
    
    type PointVertex =
        {
            [<Position>] pos : V4d
            [<Color; Interpolation(InterpolationMode.Flat)>] col : V4d
            //[<Normal>] n : V3d
            [<Semantic("ViewCenter"); Interpolation(InterpolationMode.Flat)>] vc : V3d
            [<Semantic("ViewPosition")>] vp : V3d
            [<Semantic("AvgPointDistance")>] dist : float
            [<Semantic("DepthRange"); Interpolation(InterpolationMode.Flat)>] depthRange : float
            [<PointSize>] s : float
            [<Semantic("PointPixelSize")>] ps : float
            [<PointCoord>] c : V2d
            [<Normal>] n : V3d
            [<Semantic("TreeId")>] id : int
            [<Semantic("MaxTreeDepth")>] treeDepth : int
            [<FragCoord>] fc : V4d
            [<SamplePosition>] sp : V2d
        }


    let flipNormal (n : V3d) =
        let n = Vec.normalize n


        //let a = Vec.dot V3d.IOO n |> abs
        //let b = Vec.dot V3d.OIO n |> abs
        //let c = Vec.dot V3d.OOI n |> abs

        0.5 * (n + V3d.III)

        //let mutable n = n
        //let x = n.X
        //let y = n.Y
        //let z = n.Z


        //let a = abs (atan2 y x)
        //let b = abs (atan2 z x)
        //let c = abs (atan2 z y)

        //let a = min a (Constant.Pi - a) / Constant.PiHalf |> clamp 0.0 1.0
        //let b = min b (Constant.Pi - b) / Constant.PiHalf |> clamp 0.0 1.0
        //let c = min c (Constant.Pi - c) / Constant.PiHalf |> clamp 0.0 1.0

        //let a = sqrt (1.0 - a*a)
        //let b = sqrt (1.0 - b*b)
        //let c = sqrt (1.0 - c*c)


        //let a = a + Constant.Pi
        //let b = b + Constant.Pi
        //let c = c + Constant.Pi
        



        

        //V3d(a,b,c)
        //if x > y then
        //    if z > x then
        //        n <- n / n.Z
        //    else
        //        n <- n / n.X
        //else
        //    if z > y then 
        //        n <- n / n.Z
        //    else
        //        n <- n / n.Y

        //0.5 * (n + V3d.III)


        //let n = Vec.normalize n
        //let theta = asin n.Z
        //let phi = atan (abs (n.Y / n.X))
        
        //let theta =
        //    theta + Constant.PiHalf
        //V3d(phi / Constant.PiHalf,theta / Constant.Pi, 1.0)
        //if x > y then
        //    if z > x then
        //        if n.Z < 0.0 then -n
        //        else n
        //    else
        //        if n.X < 0.0 then -n
        //        else n
        //else
        //    if z > y then 
        //        if n.Z < 0.0 then -n
        //        else n
        //    else
        //        if n.Y < 0.0 then -n
        //        else n


    let getNdcPointRadius (vp : V4d) (dist : float) =
        let ppx = uniform.ProjTrafo * (vp + V4d(0.5 * dist, 0.0, 0.0, 0.0))
        let ppy = uniform.ProjTrafo * (vp + V4d(0.0, 0.5 * dist, 0.0, 0.0))
        let ppz = uniform.ProjTrafo * vp

        let ppz = ppz.XYZ / ppz.W
        let d1 = ppx.XYZ / ppx.W - ppz |> Vec.length
        let d2 = ppy.XYZ / ppy.W - ppz |> Vec.length
        0.5 * (d1 + d2)
        
    let div (v : V4d) = v.XYZ / v.W



    [<GLSLIntrinsic("gl_InvocationID")>]
    let idx() = onlyInShaderCode ""

    let lodPointSize2 (v : PointVertex) =
        vertex { 
            let mv = 
                if idx() = 0 then uniform.LeftModelViewTrafos.[v.id]
                else uniform.RightModelViewTrafos.[v.id]

            let vp = mv * v.pos
            let vn = mv * V4d(v.n, 0.0) |> Vec.xyz
            let pp = div (uniform.ProjTrafo * vp)

            let pixelSize = uniform.Scales.[v.id].X  * uniform.PointSize
            let ndcRadius = pixelSize / V2d uniform.ViewportSize

            let vpx = uniform.ProjTrafoInv * (V4d(pp.X + ndcRadius.X, pp.Y, pp.Z, 1.0)) |> div
            let vpy = uniform.ProjTrafoInv * (V4d(pp.X, pp.Y + ndcRadius.Y, pp.Z, 1.0)) |> div
            let dist = 0.5 * (Vec.length (vpx - vp.XYZ) + Vec.length (vpy - vp.XYZ))

            let vpz = vp + V4d(0.0, 0.0, 0.5*dist, 0.0)
            let fpp = uniform.ProjTrafo * vpz
            let opp = uniform.ProjTrafo * vp
        
            let pp0 = opp.XYZ / opp.W
            let ppz = fpp.XYZ / fpp.W
        
            let depthRange = abs (pp0.Z - ppz.Z)

            let pixelSize = 
                if ppz.Z < -1.0 then -1.0
                else pixelSize
            
            let col =
                if uniform.PointVisualization &&& PointVisualization.Color <> PointVisualization.None then
                    v.col.XYZ
                else
                    V3d.III

            let o = uniform.Overlay.[v.id].X
            let h = heat (float v.treeDepth / 6.0)
            let col =
                if uniform.PointVisualization &&& PointVisualization.OverlayLod <> PointVisualization.None then
                    o * h.XYZ + (1.0 - o) * col
                else
                    col

            let pixelSize = 
                if uniform.PointVisualization &&& PointVisualization.Antialias <> PointVisualization.None then pixelSize + 1.0
                else pixelSize

            return { v with ps = float (int pixelSize); n = vn; s = pixelSize; pos = fpp; depthRange = depthRange; vp = vpz.XYZ; vc = vpz.XYZ; col = V4d(col, v.col.W) }
        }



    type Fragment =
        {
            [<Color>] c : V4d
            [<Depth(DepthWriteMode.OnlyGreater)>] d : float
        }

    let lodPointCircular (v : PointVertex) =
        fragment {
            let mutable cc = v.c
            let c = v.c * 2.0 - V2d.II
            let f = Vec.dot c c - 1.0
            if f > 0.0 then discard()
            
            let t = 1.0 - sqrt (max 0.0 -f)
            let depth = v.fc.Z
            let outDepth = depth + v.depthRange * t
            
            let mutable alpha = v.col.W
            let mutable color = v.col.XYZ

            if uniform.PointVisualization &&& PointVisualization.Antialias <> PointVisualization.None then
                let dx = ddx(v.c) * 2.0
                let dy = ddy(v.c) * 2.0
                let dfx = 2.0*c.X*dx.X + 2.0*c.Y*dx.Y
                let dfy = 2.0*c.X*dy.X + 2.0*c.Y*dy.Y
                let x = sqrt (dfx * dfx + dfy * dfy)
                let d = if x < 0.0001 || f >= -0.00001 then 0.0 else abs f / x
                alpha <- min 1.0 (d / 2.0)
                
            if uniform.PointVisualization &&& PointVisualization.FancyPoints <> PointVisualization.None then
                let vd = heat(v.ps / 8.0).XYZ
                color <- 0.5 * (vd + V3d.III)
                
            if uniform.PointVisualization &&& PointVisualization.Lighting <> PointVisualization.None then
                let c = 
                    if uniform.PointVisualization &&& PointVisualization.Antialias <> PointVisualization.None then
                        c * (1.0 + 2.0 / v.ps)
                    else
                        c

                let f = Vec.dot c c
                let diffuse = sqrt (max 0.0 (1.0 - f))
                color <- color * diffuse

            return { c = V4d(color, alpha); d = outDepth }
        }


[<EntryPoint;STAThread>]
let main argv = 
    Aardvark.Init()

    let win = 
        window {
            display Display.OpenVR
            
        }
               
    //let sg =
    //    Sg.box' C4b.Red Box3d.Unit
    //    |> Sg.shader {
    //        do! DefaultSurfaces.trafo
    //        do! DefaultSurfaces.simpleLighting
    //    }

    let store = @"C:\Users\aszabo\bla\Eltville\2020078-Eltville_store"
    let key = Path.combine [(if System.IO.Directory.Exists store then store else System.IO.Path.GetDirectoryName store);"key.txt"] |> File.readAllText
    let inst = Importy.myImport store (new LruDictionary<_,_>(1<<<20)) key |> Option.get
    let root = (inst.root.Id :?> IPointCloudNode)
    let centroid = V3d root.CentroidLocal + root.Cell.GetCenter()
    
    let t = cval V3d.Zero
    let s = cval 1.0

    let centerTrafo = 
        Trafo3d.Translation -centroid * 
        Trafo3d.Translation (V3d.OOI*1.0) * 
        Trafo3d.Translation (V3d.OIO*3.0) * 
        Trafo3d.Translation -(V3d.IOO*1.0) * 
        Trafo3d.Scale 1.0 *
        Trafo3d.Identity

    let cfg = { LodTreeRenderConfig.simple with views = win.View; projs = win.Proj }

    let sg = 
        Sg.lodTree cfg (ASet.single inst)
        |> Sg.shader {
            do! PointSetShaders.lodPointSize2
            do! PointSetShaders.lodPointCircular
        }
        |> Sg.trafo' centerTrafo
        |> Sg.translation t
        |> Sg.scaling (s |> AVal.map ((*)V3d.III))
        |> Sg.uniform "PointSize" (AVal.constant 5.0)
        |> Sg.uniform "ViewportSize" win.Sizes
        |> Sg.uniform "PointVisualization" (PointVisualization.Color |> AVal.constant)
        |> Sg.uniform "MagicExp" (~~0.0)
        |> Sg.uniform "ViewTrafo" win.View
        |> Sg.uniform "ProjTrafo" win.Proj

    //let combine (view : Trafo3d[]) (proj : Trafo3d[]) =
    //    let v = view.[0].Forward.Orthonormalized()
    //    Trafo3d(v,v.Inverse), proj.[0]
    //    //let v =
    //    //    let c = view |> Array.map (fun v -> v.Backward.C3.XYZ) |> Array.average
    //    //    let f = view |> Array.map (fun v -> -v.Backward.C2.XYZ) |> Array.average
    //    //    let u = view |> Array.map (fun v -> v.Backward.C1.XYZ) |> Array.average
    //    //    CameraView.lookAt (c-f*0.1) (c+f) u |> CameraView.viewTrafo
    //    //let p =
    //    //    Frustum.perspective 130.0 0.1 1000.0 1.0 |> Frustum.projTrafo
    //    //v,p
            

    //let vp = AVal.map2 combine win.View win.Proj
    //sg?ViewTrafo <- AVal.map fst vp
    //sg?ProjTrafo <- AVal.map snd vp

    let thread =
        startThread <| fun () ->
            while true do
                let l = Console.ReadKey()
                printfn "%A" l
                let v = win.View |> AVal.force
                let f = v.[0].Backward.C2.XYZ * V3d.IIO |> Vec.normalize
                let r = V3d(f.Y, -f.X, 0.0)
                let u = V3d.OOI
                match l.KeyChar with
                | 'd' -> transact (fun () -> t.Value <- t.Value + 0.5 * r)
                | 'a' -> transact (fun () -> t.Value <- t.Value - 0.5 * r)
                | 'w' -> transact (fun () -> t.Value <- t.Value + 0.5 * f)
                | 's' -> transact (fun () -> t.Value <- t.Value - 0.5 * f)
                | 'e' -> transact (fun () -> t.Value <- t.Value + 0.25 * u)
                | 'c' -> transact (fun () -> t.Value <- t.Value - 0.25 * u)
                | 't' -> transact (fun () -> s.Value <- s.Value + 0.02)
                | 'r' -> transact (fun () -> s.Value <- s.Value - 0.02)
                | _ -> ()



    win.Scene <- sg
    win.Run()
    0
