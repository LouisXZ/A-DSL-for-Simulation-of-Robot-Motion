
module RobotM where

import Prelude hiding (iterate,seq)
import Data.List hiding (iterate,find)
import Control.Monad.Trans
import Control.Monad.State
import Control.Monad.Except
import Control.Monad
import Data.Char

type Pos = (Int,Int)
type Item = String

type World = [(Pos,Maybe [Item])]
type S = (Pos,[Item],World,[Pos])

type Step a =  ExceptT Err (State S) a
type Prog = [Step ()]


data Dir = Rt|Lt|Up|Dn deriving (Show,Eq)
dir :: Dir -> Pos -> Pos
dir Rt (x,y) = (x+1,y)
dir Lt (x,y) = (x-1,y)
dir Up (x,y) = (x,y-1)
dir Dn (x,y) = (x,y+1)

data Err = BlockedAt    Pos
         | CannotPickAt Pos
         | CannotPutAt  Pos
         | Unreachable  Pos
  deriving (Eq)

instance Show Err where
   show (BlockedAt p)    = "Blocked at " ++ show p
   show (CannotPickAt p) = "No item can be picked up at " ++ show p
   show (CannotPutAt p)  = "No item can be put down at " ++ show p
   show (Unreachable p)  = show p ++ " is a wall or unreachable"


-- | the position is available

isClear :: Pos -> S -> Bool
isClear p (_,_,w,_) = case lookup p w of
                       Just (Just _) -> True
                       otherwise     -> False

-- | the position has item(s)

hasItem :: S -> Bool
hasItem (p,_,w,_) = case lookup p w of
                    Just (Just xs) -> if length xs > 0 then True else False
                    otherwise      -> False

-- | check whether the bag of the robot is empty or not

bagEmpty :: S -> Bool
bagEmpty (_,i,_,_) = case length i of 
                    0 -> True
                    _ -> False

-- | move function
move :: Dir -> Step ()
move d = do (p,i,w,ps) <- lift get
            let dp = dir d p
            case isClear dp (p,i,w,ps) of
              True  -> lift $ put (dp,i,w,if ps==[] then dp:p:ps else dp:ps)
              False -> throwError $ BlockedAt dp

-- | pick up an item
pick :: Step ()
pick = do (p,i,w,ps) <- lift get
          case hasItem (p,i,w,ps) of
            True  -> case lookup p w of
                       Just (Just (y:ys)) -> lift $ put (p,y:i,decW p w,ps)
            False -> throwError $ CannotPickAt p

-- | put down an item
putItem :: Step ()
putItem = do (p,i,w,ps) <- lift get    
             case i of 
               []     -> throwError $ CannotPutAt p
               (n:ns) -> lift $ put (p,ns,incW p n w,ps)
                                                                                      

-- | remove an item from the World
decW :: Pos -> World -> World
decW _ [(wp,Nothing)] = [(wp,Nothing)]
decW p w              = case (findIndex (==p) (map fst w)) of
                          Just n -> case w!!n of
                                      (wp,Just (x:xs)) -> case splitAt n w of 
                                                            (front,back) -> front ++ (wp,Just xs):tail back
                          Nothing -> w

-- | increase an item in the World                     
incW :: Pos -> String -> World -> World
incW _ _ [(wp,Nothing)] = [(wp,Nothing)]
incW p str w            = case (findIndex (==p) (map fst w)) of
                            Just n -> case w!!n of
                                        (wp,Just xs) -> case splitAt n w of  
                                                          (front,back) -> front ++ (wp,Just (str:xs)):tail back
                            Nothing -> w



-- | combinators to combine robot actions.

-- | while condition

while :: (S -> Bool) -> Step () -> Step ()
while f g = do s <- lift get
               case f s of 
                 True  -> g 
                 False -> return ()

-- | if condition

iff :: (S -> Bool) -> Step () -> Step () -> Step ()
iff f g h = do s <- lift get
               if f s then g else h


-- | iterate n times operation

iterate :: Int -> Step () -> Step ()
iterate n f | n>0       = f >> iterate (n-1) f               
            | otherwise = return ()


-- | try the first operation, if it fails then do the second operation.

try :: Step () -> Step () -> Step ()
try f g = do s <- lift get
             catchError f (\_ -> lift (put s) >> g)

-- | run a list of operations 

prog :: Prog -> S -> (Either Err (), S)
prog []     s = (Right (),s)
prog (x:xs) s = case runState (runExceptT x) s of
                  (Right (),s') -> prog xs s'
                  (Left err,s') -> (Left err,s')


-- | run a step ()

runstep :: Step () -> S -> IO ()
runstep step s = grid $ runState (runExceptT step) s


-- | a path finding strategy

noop :: Step ()
noop = return ()

unreachable :: Pos -> Step ()
unreachable = \p -> throwError $ Unreachable p

-- | find a path
path :: Pos -> Pos -> Step ()
path p q = do (rp,i,w,h) <- lift get
              lift $ put (p,i,w,h)
              let s = (p,i,w,h) in
                case isClear p s && isClear q s of
                  True  -> find [] p q
                  False -> case not (isClear p s) of
                             True  -> unreachable p
                             False -> unreachable q


-- | helper function
trylist :: [Pos] -> Pos -> Pos -> [Dir] -> Step ()
trylist _  _ q []     = unreachable q
trylist ps p q (x:xs) = try (move x >> (find (p:ps) (dir x p) q)) (trylist ps p q xs)

-- | helper function
getmoves :: Pos -> Pos -> [Dir]
getmoves p q | rtof p q = [Rt, Up, Dn, Lt]
             | ltof p q = [Lt, Up, Dn, Rt]
             | upof p q = [Up, Rt, Lt, Dn]
             | dnof p q = [Dn, Rt, Lt, Up]
             | neof p q = [Up, Rt, Lt, Dn]
             | swof p q = [Dn, Lt, Rt, Up]
             | nwof p q = [Up, Lt, Rt, Dn]
             | seof p q = [Dn, Rt, Lt, Up]  

-- | helper function
find :: [Pos] -> Pos -> Pos -> Step ()
find ps p q | p==q              = noop
            | not (p `elem` ps) = trylist ps p q $ getmoves p q
            | otherwise         = unreachable q


-- | Helper functions

rtof :: Pos -> Pos -> Bool
rtof (x,y) (m,n) | x<m && y==n = True
                 | otherwise   = False

ltof :: Pos -> Pos -> Bool
ltof (x,y) (m,n) | x>m && y==n = True
                 | otherwise   = False

upof :: Pos -> Pos -> Bool
upof (x,y) (m,n) | x==m && y>n = True
                 | otherwise   = False

dnof :: Pos -> Pos -> Bool
dnof (x,y) (m,n) | x==m && y<n = True
                 | otherwise   = False

neof :: Pos -> Pos -> Bool
neof (x,y) (m,n) | x<m && y>n = True
                 | otherwise  = False

swof :: Pos -> Pos -> Bool
swof (x,y) (m,n) | x>m && y<n = True
                 | otherwise  = False

nwof :: Pos -> Pos -> Bool
nwof (x,y) (m,n) | x>m && y>n = True
                 | otherwise  = False

seof :: Pos -> Pos -> Bool
seof (x,y) (m,n) | x<m && y<n = True
                 | otherwise  = False


-- | World, Testing, Printing

exWorld :: World
exWorld = [((1,1),Just ["Apple"]),((2,1),Just []),((3,1),Just ["Language"]),((4,1),Just []),((5,1),Just ["Hello","Kitty"]),((6,1),Just []),((7,1),Just []),((8,1),Just []),
            ((1,2),Nothing), ((2,2),Just []), ((3,2),Nothing), ((4,2),Nothing), ((5,2),Nothing), ((6,2),Nothing), ((7,2),Just []),((8,2),Just []),
            ((1,3),Just []), ((2,3),Just []), ((3,3),Just ["Ice Cream"]), ((4,3),Just []), ((5,3),Just []), ((6,3),Just []), ((7,3),Just ["Dragon"]),((8,3),Just []),
            ((1,4),Nothing), ((2,4),Nothing), ((3,4),Nothing), ((4,4),Nothing), ((5,4),Just []), ((6,4),Nothing), ((7,4),Nothing),((8,4),Just []),
            ((1,5),Just []), ((2,5),Just []), ((3,5),Just []), ((4,5),Just []), ((5,5),Just []), ((6,5),Nothing), ((7,5),Just []),((8,5),Just []),
            ((1,6),Just []), ((2,6),Just ["Orange"]), ((3,6),Just []), ((4,6),Nothing), ((5,6),Nothing), ((6,6),Just []), ((7,6),Just []),((8,6),Just []),
            ((1,7),Just ["Cheese"]), ((2,7),Nothing), ((3,7),Just []), ((4,7),Just ["Island"]), ((5,7),Just []), ((6,7),Just []), ((7,7),Nothing),((8,7),Just []),
            ((1,8),Nothing), ((2,8),Just []), ((3,8),Nothing), ((4,8),Just []), ((5,8),Just []), ((6,8),Just ["Coconut"]), ((7,8),Just []),((8,8),Just [])
           ]

exState :: S
exState = ((3,1),["Blueberry"],exWorld,[])


type PWorld = [(Pos, String)]

-- | convert a World to a list of signs

pathworld :: S -> PWorld
pathworld (_,_,[],_)                      = []
pathworld ((rx,ry),ri,(((x,y),wi):xs),ps) = case wi of
                          Just [] -> case (x,y)==(rx,ry) of
                                       True  -> ((x,y),"R") : pathworld ((rx,ry),ri,xs,ps) 
                                       False -> case (x,y) `elem` ps of
                                                  True  -> ((x,y),"@") : pathworld ((rx,ry),ri,xs,ps)
                                                  False -> ((x,y),"_") : pathworld ((rx,ry),ri,xs,ps)
                          Just _  -> case (x,y)==(rx,ry) of
                                       True  -> ((x,y),"R") : pathworld ((rx,ry),ri,xs,ps)
                                       False -> case (x,y) `elem` ps of
                                                  True  -> ((x,y),"@") : pathworld ((rx,ry),ri,xs,ps)
                                                  False -> ((x,y),"*") : pathworld ((rx,ry),ri,xs,ps)
                          Nothing -> case (x,y)==(rx,ry) of
                                       True  -> ((x,y),"R") : pathworld ((rx,ry),ri,xs,ps)
                                       False -> case (x,y) `elem` ps of
                                                  True  -> ((x,y),"@") : pathworld ((rx,ry),ri,xs,ps)
                                                  False -> ((x,y),"#") : pathworld ((rx,ry),ri,xs,ps)

-- | print each line of the PWorld list
printLn :: (Int,Int) -> PWorld -> [String]
printLn _ []     = []
printLn (c,r) pw = (["\n"] ++ ["|"] ++ (take c . map snd) pw ++ ["|"] ++ printLn (c,r) (drop c pw))


-- | add number (top and left side)

addNum :: [String] -> [String] -> [String]
addNUm _ [] = []
addNum [] xs = xs
addNum (y:ys) xs = case elemIndex "\n" xs of
                     Just n -> case splitAt (n+1) xs of
                                 (front,back) -> front ++ [y] ++ addNum ys back
                     Nothing -> xs
                                     
-- | calculate the number of row and column
                             
countW :: PWorld -> (Int,Int)
countW [] = (0,0)
countW (((x,y),i):xs) = let mx = max x (fst (countW xs)) in 
                                       let my = max y (snd (countW xs)) in 
                                           (mx,my)


nprint :: PWorld -> IO()
nprint [] = putStrLn "Nothing to print !"
nprint (((x,y),i):((x',y'),i'):xs) = let mx = max x (fst (countW xs)) in 
                                       let my = max y (snd (countW xs)) in 
                                          let (c,r) = (mx,my) in
                                             let pw = (((x,y),i):((x',y'),i'):xs) in
                                               pci (["  "] ++ (map show) [0..c+1] ++ ["X"]) >>
                                               pci ([" 0"] ++ replicate (c+2) "-") >>
                                               pci ([""] ++ (map show) [1] ++ ["|"] ++ (take c . map snd) pw ++ ["|"] ++ (addNum ((map show) [2..r]) (printLn (c,r) (drop c pw)))) >>
                                               pci ([""] ++ map show [r+1] ++ replicate (c+2) "-") >>
                                               pci ([" Y"])
                                                 where pci = (putStrLn . concat . (intersperse " "))

-- | print a grid

grid :: (Either Err (), S) -> IO ()
grid (Right (), s)   = (nprint . pathworld) s
                     >> pp (Right (), s)
grid (Left (str), s) = grid (Right (), s)
                     >> putStrLn ("ErrorInfo: " ++ show str ++ " !")


-- | only print Step and Error Information

nogrid :: (Either Err (), S) -> IO()
nogrid (Right (), s)   = pp (Right (), s)
nogrid (Left (str), s) = pp (Left (str), s) >> putStrLn ("ErrorInfo: " ++ show str ++ " !")

-- | a helper function in printing a grid
pp ::(Either Err (), S) -> IO()
pp (str, (rp,i,w,ps)) = putStrLn ("Robot Pos: " ++ show rp)
                      >> putStrLn ("Robot Has: " ++ show i)
                      >> putStrLn (show rp ++ " Has: " ++ show ( case lookup rp w of
                                                                      Just (Just xs) -> xs))
                      >> putStrLn ("Step:      " ++ if length ps == 0 then show 0 else (show . (+(-1)). length) ps)



--   grid $ prog [iff bagEmpty pick putItem] exState
--   grid $ prog [while hasItem (move Lt), try (move Up) (iterate 2 (move Lt))] exState
--   grid $ prog [path (7,8) (1,1), move Rt] exState
--   
--   grid $ prog [] exState
--   grid $ prog [(move Dn)] exState
--   grid $ prog [move Lt, iterate 2 (move Dn)] exState
--   grid $ prog [iterate 2 pick] exState
--   grid $ prog [while hasItem (iterate 2 (move Rt))] exState
--   grid $ prog [iff bagEmpty (move Lt) (move Rt)] exState
--   grid $ prog [try (move Dn) (move Lt)] exState
--   grid $ prog [iff hasItem putItem (move Lt)] exState
-- 
--   grid $ prog [path (3,1) (6,6)] exState
--   grid $ prog [path (3,1) (2,5)] exState
--   grid $ prog [path (7,8) (1,1)] exState
--   grid $ prog [path (7,8) (1,1), move Rt, iterate 2 (move Dn)] exState
--   grid $ prog [path (7,8) (1,1), move Rt, iterate 2 (move Dn),iterate 2 (move Up),move Rt] exState
--   grid $ prog [path (3,1) (1,6), move Dn] exState
--   grid $ prog [path (3,1) (1,7)] exState
--   grid $ prog [path (4,1) (1,7)] exState
-- nogrid $ prog [path (7,8) (1,7), move Rt] exState
--  


